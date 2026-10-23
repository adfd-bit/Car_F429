/**
 * @file    ldrobot_stl.c
 * @brief   乐动 STL-06N(STL-06NBJ) 激光雷达：USART6 接收、协议解析、聚类、滤波
 *
 * 数据流总览：
 *   USART6 中断（HAL_UARTEx_ReceiveToIdle_IT）
 *        │  ldrobot_stl_rx_event_callback()：把本次收到的字节压入软件环形缓冲区
 *        ▼
 *   rx_ring[]（软件环形缓冲区，4 KB）
 *        │  ldrobot_stl_process()：主循环取出字节
 *        ▼
 *   feed_byte()：按 0x54 0x2C 帧头拼出 47 字节完整帧
 *        │
 *        ▼
 *   parse_measure_frame()：CRC8 校验 → 首尾角插值 → 逐点过滤
 *        │
 *        ▼
 *   collect_point() / add_group()：相邻点聚类成障碍物
 *        │  角度差 < 3° 且两点距离 < 150 mm → 同一障碍物
 *        ▼
 *   publish_scan()：每跨过 0° 发布上一圈
 *        │  组内均值 → 点数过滤 → 时域滤波 → 按距离排序
 *        ▼
 *   snapshot：由 ldrobot_stl_get_scan() / ldrobot_stl_get_obstacles() 取出
 *
 * 线程/中断安全：
 *   - 环形缓冲区是单生产者（USART6 中断）/单消费者（主循环）模型；
 *     ISR 只写 rx_head，主循环只写 rx_tail，16 位索引在 Cortex-M4 上读写原子。
 *   - 其余解析状态只在主循环里访问，不需要加锁。
 */

#include "ldrobot_stl.h"
#include "usart.h"

#include <math.h>
#include <string.h>

/* ============================= 内部常量 ============================= */

#define LDR_STL_PI_F        3.14159265358979323846f
#define LDR_STL_DEG2RAD     (LDR_STL_PI_F / 180.0f)
#define LDR_STL_RAD2DEG     (180.0f / LDR_STL_PI_F)

/* 角度容差：首尾角插值时角度可能正好等于 360°，归一化时容差内直接按 0 处理 */
#define LDR_STL_ANGLE_EPS   0.01f

/* ============================= 内部类型 ============================= */

/* 换算到小车坐标系后的一个测距点：
 * x 向右、y 向前（单位 mm，浮点保留精度）；
 * angle 保留雷达原始角度（未加安装角），聚类时用它判断相邻点是否连续 */
typedef struct {
    float x;
    float y;
    float angle;
} scan_point_t;

/* 一个"连续目标"分组的累加器：
 * first/last 用于判断相邻点连续性，
 * sum_x/sum_y 用于最后求障碍物中心（组内平均值），
 * samples 用于点数过滤和置信度输出 */
typedef struct {
    scan_point_t first;
    scan_point_t last;
    float    sum_x;
    float    sum_y;
    uint16_t samples;
} point_group_t;

/* =========================== 运行状态变量 =========================== */

/* ---- 软件环形缓冲区 ---- */
static uint8_t  rx_chunk[LDR_STL_RX_CHUNK_SIZE]; /* HAL 接收临时块（中断写入） */
static uint8_t  rx_ring[LDR_STL_RING_SIZE];      /* 软件环形缓冲区 */
static volatile uint16_t rx_head;                /* 写指针：只由中断修改 */
static volatile uint16_t rx_tail;                /* 读指针：只由主循环修改 */
static volatile uint32_t rx_dropped;             /* 累计丢弃字节数（缓冲区满） */
static volatile bool     rx_error_pending;       /* 中断里发生串口错误，主循环处理 */
static uint32_t          observed_dropped;       /* 主循环已处理的丢字节计数 */

/* ---- 组帧状态 ---- */
static uint8_t frame_buf[LDR_STL_FRAME_LEN]; /* 正在拼装的帧 */
static uint8_t frame_index;                  /* 已收字节数，0 = 等待帧头 */
static uint8_t frame_length;                 /* 当前帧的预期总长度 */

/* ---- 分圈状态 ---- */
static float    previous_angle;       /* 上一个有效点的角度，用于检测跨 0° */
static bool     previous_angle_valid; /* previous_angle 是否可用 */
static bool     scan_started;         /* 是否已跨过 0°、进入有效扫描圈 */
static bool     receive_active;       /* 是否正在接收数据（区分没接线和数据出错） */
static uint32_t last_frame_tick;      /* 最近一帧有效数据的时刻(ms) */
static uint32_t last_scan_tick;       /* 最近一圈发布的时刻(ms) */
static bool     has_completed_scan;   /* 是否发布过至少一圈 */

/* ---- 统计计数（供诊断接口读取） ---- */
static uint32_t valid_frames; /* CRC 校验成功并解析的帧数 */
static uint32_t bad_frames;   /* 帧头错乱 / CRC 失败 / 数据异常的帧数 */

/* ---- 聚类状态 ---- */
static point_group_t first_group;   /* 跨 0° 处暂存的首组，等圈末判断能否与末组合并 */
static point_group_t current_group; /* 正在累积的分组 */
static bool          current_exists;/* current_group 是否有效 */
static bool          first_saved;   /* first_group 是否已暂存 */

/* ---- 正在构建的扫描结果（本圈累积） ---- */
static ldr_stl_obstacle_t building_obstacles[LDR_STL_MAX_OBSTACLES];
static uint8_t            building_count;

/* ---- 对外发布的快照（一圈结束时整体更新） ---- */
static ldr_stl_scan_t snapshot;

#if LDR_STL_ENABLE_TEMPORAL_FILTER
/* ---- 时域滤波：上一圈输出的障碍物，用于跨圈匹配和指数平滑 ---- */
static ldr_stl_obstacle_t track_prev[LDR_STL_MAX_OBSTACLES];
static uint8_t            track_prev_count;
static bool               track_prev_valid; /* 上一圈是否有有效结果 */
#endif

/* ---- 多圈确认：跨圈跟踪表 + 命中计数 ---- */
typedef struct {
    bool     used;       /* 该槽位是否正在跟踪一个目标 */
    uint8_t  hits;       /* 命中计数：命中 +1，丢失 -1，饱和于确认门限 */
    uint8_t  misses;     /* 已确认目标连续丢失的圈数（滞回用） */
    int32_t  x_mm;       /* 最近一次命中的坐标 */
    int32_t  y_mm;
    uint32_t range_mm;
    float    angle_deg;
    uint16_t points;     /* 最近一次命中的聚类点数 */
} confirm_track_t;

static confirm_track_t    confirm_tracks[LDR_STL_MAX_TRACKS];
static ldr_stl_obstacle_t confirmed_obstacles[LDR_STL_MAX_OBSTACLES];
static uint8_t            confirmed_count;

/* ======================= CRC8 查表（多项式 0x4D） =======================
 * 来源：乐动 STL-06N 开发手册 / LDROBOT SDK（MIT License）。
 * 初值 0，MSB-first，不反射。 */
static const uint8_t ldr_crc_table[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
    0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
    0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
    0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
    0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
    0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
    0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
    0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
    0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
    0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
    0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
    0x7f, 0x32, 0xe5, 0xa8
};

/* ============================== 小工具 ============================== */

/* 小端读取 16 位无符号数（低字节在前），雷达协议使用小端 */
static uint16_t read_le_u16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[1] << 8) | p[0]);
}

/* 查表计算 CRC8，len 为参与校验的字节数（不含末尾校验字节本身） */
static uint8_t ldr_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0U; /* CRC 初值为 0 */
    while (len-- != 0U) {
        crc = ldr_crc_table[crc ^ *data++];
    }
    return crc;
}

/* 角度归一化到 [0,360) */
static float normalize_deg(float angle)
{
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f) angle += 360.0f;
    return angle;
}

#if LDR_STL_BODY_MASK_ENABLE
/* angle 是否落在 [start, end] 扇区内。start > end 时表示跨 0°，
 * 例如 350°~20° 会匹配 355° 和 5°，但不匹配 180°。 */
static bool angle_in_sector(float angle, float start, float end)
{
    angle = normalize_deg(angle);
    start = normalize_deg(start);
    end   = normalize_deg(end);

    if (start <= end) {
        return (angle >= start) && (angle <= end);
    }
    return (angle >= start) || (angle <= end);
}
#endif

/* 浮点毫米转整数毫米，正负对称四舍五入 */
static int32_t round_mm(float value)
{
    return (int32_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

/* 把角度差折算到 [-180,180]，用于跨 0° 的角度比较 */
static float wrap180_deg(float delta)
{
    while (delta > 180.0f) delta -= 360.0f;
    while (delta < -180.0f) delta += 360.0f;
    return delta;
}

/* ========================== 软件环形缓冲区 ========================== */

/* 从缓冲区取 1 个字节，成功返回 true（主循环调用） */
static bool ring_pop(uint8_t *byte)
{
    uint16_t tail = rx_tail;
    if (tail == rx_head) return false; /* 空 */
    *byte = rx_ring[tail];
    rx_tail = (uint16_t)((tail + 1U) & (LDR_STL_RING_SIZE - 1U));
    return true;
}

/* =========================== 坐标与聚类 =========================== */

/* 把"雷达角度 + 距离"换算成小车坐标系下的点（单位 mm）：
 *   x = 右偏安装量 + r·sin(θ + 安装角)
 *   y = 前偏安装量 + r·cos(θ + 安装角)
 * 角度定义 0° 指向车头、顺时针增大，所以 90° 在右侧 */
static scan_point_t point_for_angle(float angle, uint16_t distance)
{
    scan_point_t point;
    float radians = (angle + LDR_STL_MOUNT_HEADING_DEG) * LDR_STL_DEG2RAD;

    point.x = (float)distance * sinf(radians) + (float)LDR_STL_MOUNT_RIGHT_MM;
    point.y = (float)distance * cosf(radians) + (float)LDR_STL_MOUNT_FORWARD_MM;
    point.angle = angle; /* 保留雷达原始角度，供连续性判断 */
    return point;
}

/* 判断相邻两点是否属于同一障碍物：
 *   - 角度间隔 ≤ 3°（across_zero=true 时用于跨 0° 的接缝，需要补 360°）
 *   - 两点直线距离 ≤ 150 mm
 * 两个条件同时满足才算连续。 */
static bool points_connected(const scan_point_t *a, const scan_point_t *b,
                             bool across_zero)
{
    /* 跨 0° 时把 b 的角度加 360°，避免 359°→1° 被算成 -358° */
    float gap = across_zero ? (360.0f + b->angle - a->angle)
                            : (b->angle - a->angle);
    float dx = b->x - a->x;
    float dy = b->y - a->y;

    return (gap >= 0.0f) && (gap <= LDR_STL_GROUP_ANGLE_GAP_DEG) &&
           ((dx * dx + dy * dy) <=
            (LDR_STL_GROUP_DISTANCE_MM * LDR_STL_GROUP_DISTANCE_MM));
}

/* 以给定点作为首点，开始一个新分组 */
static void group_start(point_group_t *group, scan_point_t point)
{
    group->first = point;
    group->last = point;
    group->sum_x = point.x;
    group->sum_y = point.y;
    group->samples = 1U;
}

/* 把点追加到分组末尾，同时累加坐标和与采样数 */
static void group_append(point_group_t *group, scan_point_t point)
{
    group->last = point;
    group->sum_x += point.x;
    group->sum_y += point.y;
    group->samples++;
}

/* 根据障碍物中心的 x/y 重新计算距离和方位角（时域滤波后坐标会变） */
static void update_obstacle_polar(ldr_stl_obstacle_t *obstacle)
{
    float x = (float)obstacle->x_mm;
    float y = (float)obstacle->y_mm;

    obstacle->range_mm = (uint32_t)round_mm(sqrtf(x * x + y * y));

    /* atan2(x, y)：0° 指向 +Y（车头），顺时针增大 */
    float angle = atan2f(x, y) * LDR_STL_RAD2DEG;
    if (angle < 0.0f) angle += 360.0f;
    obstacle->angle_deg = angle;
}

/* 一个分组结束时调用，把分组转成一个障碍物放进本圈结果：
 *   - 少于 LDR_STL_MIN_GROUP_POINTS 个点的组按噪点丢弃
 *   - 用组内所有点的坐标平均值作为障碍物中心（相当于一次空间平滑）
 *   - 结果超过上限时保留离车体更近的那个 */
static void add_group(const point_group_t *group)
{
    if (group->samples < LDR_STL_MIN_GROUP_POINTS) return;

    ldr_stl_obstacle_t obstacle;

    /* 组内坐标平均值：可见表面的中心，不是物体的几何中心 */
    obstacle.x_mm = round_mm(group->sum_x / (float)group->samples);
    obstacle.y_mm = round_mm(group->sum_y / (float)group->samples);
    obstacle.points = group->samples;
    update_obstacle_polar(&obstacle);

    if (building_count < LDR_STL_MAX_OBSTACLES) {
        building_obstacles[building_count++] = obstacle;
    } else {
        /* 数组已满：找当前最远的一个，比它近就替换，保证输出的是最近的障碍物 */
        uint8_t farthest = 0U;
        for (uint8_t i = 1U; i < LDR_STL_MAX_OBSTACLES; ++i) {
            if (building_obstacles[i].range_mm >
                building_obstacles[farthest].range_mm) {
                farthest = i;
            }
        }
        if (obstacle.range_mm < building_obstacles[farthest].range_mm) {
            building_obstacles[farthest] = obstacle;
        }
    }
}

/* 逐点输入的分组状态机：
 * 与当前组末点相连则并入，否则结束当前组并另起一组。
 * 圈末与圈首可能在 0° 处属于同一目标，所以第一个分组先暂存，
 * 等 finish_groups() 在圈末判断接缝能否合并。 */
static void collect_point(scan_point_t point)
{
    if (!current_exists) {
        /* 还没有分组：以这个点开第一组 */
        group_start(&current_group, point);
        current_exists = true;
    } else if (points_connected(&current_group.last, &point, false)) {
        /* 与组末点连续：并入当前组 */
        group_append(&current_group, point);
    } else {
        /* 与当前组断开：结束当前组，并以新点另起一组 */
        if (!first_saved) {
            /* 跨 0° 的首组先暂存，等圈末检查能否和最后一组合并 */
            first_group = current_group;
            first_saved = true;
        } else {
            /* 其他分组立即结算 */
            add_group(&current_group);
        }
        group_start(&current_group, point);
    }
}

/* 一圈数据结束时收尾分组，处理跨 0° 的接缝：
 *   - 只有一个分组：直接输出
 *   - 首组与末组在 0° 处相连：合并成一个目标再输出
 *   - 不相连：两个目标分别输出 */
static void finish_groups(void)
{
    if (!current_exists) return;

    if (!first_saved) {
        /* 整圈只有一个组 */
        add_group(&current_group);
    } else if (points_connected(&current_group.last, &first_group.first, true)) {
        /* 首尾在 0° 处相连：合并坐标和、点数 */
        current_group.sum_x += first_group.sum_x;
        current_group.sum_y += first_group.sum_y;
        current_group.samples = (uint16_t)(current_group.samples + first_group.samples);
        add_group(&current_group);
    } else {
        /* 首尾不相连：两个独立目标 */
        add_group(&first_group);
        add_group(&current_group);
    }
}

/* 按到坐标原点的距离由近到远排序（插入排序，最多 24 项） */
static void sort_by_range(ldr_stl_obstacle_t *list, uint8_t count)
{
    for (uint8_t i = 1U; i < count; ++i) {
        ldr_stl_obstacle_t key = list[i];
        int16_t j = (int16_t)i - 1;

        while (j >= 0 && list[j].range_mm > key.range_mm) {
            list[j + 1] = list[j];
            --j;
        }
        list[j + 1] = key;
    }
}

/* ============================ 多圈确认 ============================ */

/* 每发布一圈调用一次：
 *   - 把本圈障碍物与跟踪表按"方位角 + 距离"配对
 *   - 配对成功：刷新跟踪位置，命中计数 +1（饱和于确认门限）
 *   - 配对失败：新建跟踪（命中计数 = 1）
 *   - 本圈没配上的候选跟踪：命中计数 -1，减到 0 删除
 *   - 本圈没配上的已确认跟踪：保留 MAX_MISSES 圈的滞回后再删除
 *   - 命中计数达到 LDR_STL_CONFIRM_HITS 的跟踪写入 confirmed_obstacles
 * 同时把命中次数写回 list[i].hits，供每圈快照查看确认进度。 */
static void confirm_update(ldr_stl_obstacle_t *list, uint8_t count)
{
    bool matched[LDR_STL_MAX_TRACKS];
    memset(matched, 0, sizeof(matched));

    /* ① 本圈障碍物逐个配对 */
    for (uint8_t i = 0U; i < count; ++i) {
        int16_t best = -1;
        float best_angle_gap = 1.0e9f;

        for (uint8_t j = 0U; j < LDR_STL_MAX_TRACKS; ++j) {
            if (!confirm_tracks[j].used || matched[j]) continue;

            float angle_gap = fabsf(wrap180_deg(list[i].angle_deg -
                                               confirm_tracks[j].angle_deg));
            if (angle_gap > LDR_STL_CONFIRM_ANGLE_GAP_DEG) continue;

            float range_gap = fabsf((float)list[i].range_mm -
                                    (float)confirm_tracks[j].range_mm);
            if (range_gap > LDR_STL_CONFIRM_RANGE_GAP_MM) continue;

            if (angle_gap < best_angle_gap) {
                best_angle_gap = angle_gap;
                best = (int16_t)j;
            }
        }

        if (best >= 0) {
            /* 命中已有跟踪 */
            confirm_track_t *track = &confirm_tracks[best];
            matched[best] = true;
            track->x_mm = list[i].x_mm;
            track->y_mm = list[i].y_mm;
            track->range_mm = list[i].range_mm;
            track->angle_deg = list[i].angle_deg;
            track->points = list[i].points;
            track->misses = 0U;
            if (track->hits < LDR_STL_CONFIRM_HITS) track->hits++;
            list[i].hits = track->hits;
        } else {
            /* 新目标：优先用空槽；表满时替换命中次数最少的跟踪 */
            uint8_t slot = LDR_STL_MAX_TRACKS;
            for (uint8_t j = 0U; j < LDR_STL_MAX_TRACKS; ++j) {
                if (!confirm_tracks[j].used) { slot = j; break; }
            }
            if (slot == LDR_STL_MAX_TRACKS) {
                slot = 0U;
                for (uint8_t j = 1U; j < LDR_STL_MAX_TRACKS; ++j) {
                    if (confirm_tracks[j].hits < confirm_tracks[slot].hits) {
                        slot = j;
                    }
                }
            }

            confirm_track_t *track = &confirm_tracks[slot];
            track->used = true;
            track->hits = 1U;
            track->misses = 0U;
            track->x_mm = list[i].x_mm;
            track->y_mm = list[i].y_mm;
            track->range_mm = list[i].range_mm;
            track->angle_deg = list[i].angle_deg;
            track->points = list[i].points;
            matched[slot] = true;
            list[i].hits = 1U;
        }
    }

    /* ② 本圈未命中的跟踪：
     *    - 未确认的候选目标：命中计数 -1，减到 0 删除
     *    - 已确认目标：保留命中计数，累计连续丢失圈数，
     *      超过 LDR_STL_CONFIRM_MAX_MISSES 才删除（滞回） */
    for (uint8_t j = 0U; j < LDR_STL_MAX_TRACKS; ++j) {
        confirm_track_t *track = &confirm_tracks[j];
        if (!track->used || matched[j]) continue;

        if (track->hits >= LDR_STL_CONFIRM_HITS) {
            track->misses++;
            if (track->misses >= LDR_STL_CONFIRM_MAX_MISSES) {
                track->used = false;
            }
        } else {
            if (track->hits > 0U) track->hits--;
            if (track->hits == 0U) track->used = false;
        }
    }

    /* ③ 重建"已确认"数组，按距离由近到远排序 */
    confirmed_count = 0U;
    for (uint8_t j = 0U; j < LDR_STL_MAX_TRACKS; ++j) {
        if (!confirm_tracks[j].used ||
            confirm_tracks[j].hits < LDR_STL_CONFIRM_HITS) {
            continue;
        }
        if (confirmed_count >= LDR_STL_MAX_OBSTACLES) break;

        ldr_stl_obstacle_t *obstacle = &confirmed_obstacles[confirmed_count++];
        obstacle->x_mm = confirm_tracks[j].x_mm;
        obstacle->y_mm = confirm_tracks[j].y_mm;
        obstacle->range_mm = confirm_tracks[j].range_mm;
        obstacle->angle_deg = confirm_tracks[j].angle_deg;
        obstacle->points = confirm_tracks[j].points;
        obstacle->hits = confirm_tracks[j].hits;
    }
    sort_by_range(confirmed_obstacles, confirmed_count);
}

#if LDR_STL_ENABLE_TEMPORAL_FILTER
/* 时域滤波：把本圈障碍物与上一圈的障碍物按"方位角 + 距离"配对，
 * 配对成功后用指数平滑（EMA）更新坐标：
 *     x_new = x_prev + α · (x_scan - x_prev)
 * 这样可以明显抑制单圈测量抖动；α 越大越跟手，越小越平滑。
 * 未配对的障碍物（新出现的）直接使用本圈原始坐标。 */
static void apply_temporal_filter(ldr_stl_obstacle_t *list, uint8_t count)
{
    bool matched[LDR_STL_MAX_OBSTACLES];
    memset(matched, 0, sizeof(matched));

    if (track_prev_valid) {
        for (uint8_t i = 0U; i < count; ++i) {
            uint8_t best = 0xFFU;
            float best_angle_gap = 1.0e9f;

            /* 在上一圈结果里找方位角最近、且距离差在门限内的目标 */
            for (uint8_t j = 0U; j < track_prev_count; ++j) {
                if (matched[j]) continue;

                float angle_gap = fabsf(wrap180_deg(list[i].angle_deg -
                                                   track_prev[j].angle_deg));
                if (angle_gap > LDR_STL_TRACK_ANGLE_GAP_DEG) continue;

                float range_gap = fabsf((float)list[i].range_mm -
                                        (float)track_prev[j].range_mm);
                if (range_gap > LDR_STL_TRACK_RANGE_GAP_MM) continue;

                if (angle_gap < best_angle_gap) {
                    best_angle_gap = angle_gap;
                    best = j;
                }
            }

            if (best != 0xFFU) {
                matched[best] = true; /* 一个旧目标只能配一次 */

                float new_x = (float)track_prev[best].x_mm +
                              LDR_STL_TEMPORAL_ALPHA *
                                  ((float)list[i].x_mm - (float)track_prev[best].x_mm);
                float new_y = (float)track_prev[best].y_mm +
                              LDR_STL_TEMPORAL_ALPHA *
                                  ((float)list[i].y_mm - (float)track_prev[best].y_mm);

                list[i].x_mm = round_mm(new_x);
                list[i].y_mm = round_mm(new_y);
                update_obstacle_polar(&list[i]);
            }
        }
    }

    /* 保存本圈结果，供下一圈匹配 */
    for (uint8_t i = 0U; i < count; ++i) {
        track_prev[i] = list[i];
    }
    track_prev_count = count;
    track_prev_valid = true;
}
#endif /* LDR_STL_ENABLE_TEMPORAL_FILTER */

/* ============================ 分圈与发布 ============================ */

/* 清空"本圈累积"状态，准备重新开始一圈 */
static void clear_building(void)
{
    building_count = 0U;
    current_exists = false;
    first_saved = false;
}

/* 使当前扫描彻底失效并复位解析状态（数据流不可信时调用）：
 * discard_received=true 表示缓冲区里未解析的字节也已不可信（串口错误/溢出），
 * 直接丢掉，等下一个帧头重新同步。 */
static void invalidate_scan(bool discard_received)
{
    clear_building();

    previous_angle_valid = false;
    scan_started = false;
    receive_active = false;
    frame_index = 0U;
    frame_length = 0U;

    snapshot.valid = false;
    snapshot.count = 0U;
    snapshot.age_ms = UINT32_MAX;

#if LDR_STL_ENABLE_TEMPORAL_FILTER
    /* 数据中断过，旧结果不能再参与跨圈平滑 */
    track_prev_valid = false;
    track_prev_count = 0U;
#endif

    /* 数据流中断过，多圈确认记录也作废 */
    memset(confirm_tracks, 0, sizeof(confirm_tracks));
    confirmed_count = 0U;

    if (discard_received) {
        rx_tail = rx_head; /* 缓冲区清空 */
    }
}

/* 超时检查：超过 500 ms 没有新帧或新一圈，说明雷达断连/掉电，
 * 立即让结果失效，避免把陈旧数据当成实时障碍物。
 * 失效处理会清掉标志，因此只触发一次。 */
static void expire_scan(uint32_t now)
{
    if ((snapshot.valid &&
         (uint32_t)(now - last_scan_tick) >= LDR_STL_SCAN_TIMEOUT_MS) ||
        (receive_active &&
         (uint32_t)(now - last_frame_tick) >= LDR_STL_FRAME_TIMEOUT_MS)) {
        invalidate_scan(true);
    }
}

/* 发布一圈完整扫描：
 * 先收尾分组 → 时域滤波 → 按距离排序 → 整体写入对外快照。 */
static void publish_scan(uint32_t now)
{
    finish_groups();

#if LDR_STL_ENABLE_TEMPORAL_FILTER
    apply_temporal_filter(building_obstacles, building_count);
#endif

    sort_by_range(building_obstacles, building_count);

    /* 多圈确认：更新命中计数和"已确认"数组；
     * 同时给本圈障碍物写上 hits，随快照一起输出 */
    confirm_update(building_obstacles, building_count);

    uint32_t next_seq = snapshot.scan_seq + 1U; /* 每圈递增 */

    if (building_count > 0U) {
        memcpy(snapshot.obstacles, building_obstacles,
               (size_t)building_count * sizeof(ldr_stl_obstacle_t));
    }
    snapshot.count = building_count;
    snapshot.valid = true;
    snapshot.age_ms = 0U;
    snapshot.scan_seq = next_seq;

    last_scan_tick = now;
    has_completed_scan = true;

    /* 清空累积状态，准备接收下一圈 */
    clear_building();
}

/* ============================== 帧解析 ============================== */

/* 校验并解析一帧 47 字节测距帧（frame_buf 已填满）。
 * 成功返回 true；CRC/角度异常返回 false（由调用方计入 bad_frames）。
 *
 * 处理步骤：
 *   1. CRC8 校验
 *   2. 读取首尾角，线性插值出 12 个点的角度
 *   3. 检测角度跨 0°（一圈结束），发布上一圈
 *   4. 距离/强度过滤后交给聚类状态机 */
static bool parse_measure_frame(uint32_t now)
{
    /* ① CRC8：前 46 字节参与校验，第 47 字节是校验值 */
    if (ldr_crc8(frame_buf, LDR_STL_FRAME_LEN - 1U) !=
        frame_buf[LDR_STL_FRAME_LEN - 1U]) {
        return false;
    }

    /* 帧信息低 5 位是每帧点数，本型号固定 12 */
    if ((frame_buf[1] & 0x1FU) != LDR_STL_POINTS_PER_FRAME) {
        return false;
    }

    /* ② 起始角 / 结束角，单位 0.01°；36000 表示 0° */
    uint16_t start_raw = read_le_u16(&frame_buf[4]);
    uint16_t end_raw = read_le_u16(&frame_buf[42]);

    if (start_raw > 36000U || end_raw > 36000U) return false;

    float start = (float)(start_raw % 36000U) * 0.01f;
    float end = (float)(end_raw % 36000U) * 0.01f;

    float span = end - start;
    if (span < 0.0f) span += 360.0f; /* 跨过 0°，补一圈 */

    /* 单帧只覆盖一小段圆弧：跨度异常说明数据损坏 */
    if (span <= LDR_STL_ANGLE_EPS || span >= 180.0f) return false;

    /* ③ 逐点解析 */
    for (uint8_t i = 0U; i < LDR_STL_POINTS_PER_FRAME; ++i) {
        /* 12 个点把首尾角等分成 11 段 */
        float angle = normalize_deg(
            start + span * (float)i / (float)(LDR_STL_POINTS_PER_FRAME - 1U));

        /* 角度回绕检测：前一点接近 360°、当前点接近 0° 时视为跨 0° */
        bool wrapped = previous_angle_valid && (previous_angle - angle > 180.0f);

        if (wrapped) {
            /* 一圈结束 */
            if (scan_started) {
                publish_scan(now); /* 发布上一圈，开始新一圈 */
            } else {
                /* 上电后第一次跨 0° 只用来确定圈的起点，丢弃半圈残片 */
                clear_building();
                scan_started = true;
            }
        }

        /* 角度非回绕地倒退说明中间丢了一段数据：重新建立角度基准即可，
         * 聚类会因为角度间隔为负而自动断开，不影响后续分圈 */
        previous_angle = angle;
        previous_angle_valid = true;

        /* 还没跨过第一个 0° 时不输出结果 */
        if (!scan_started) continue;

        /* 该点距离字段的偏移：帧头 6 字节 + 每点 3 字节 */
        uint8_t offset = (uint8_t)(6U + i * 3U);
        uint16_t distance = read_le_u16(&frame_buf[offset]);
        uint8_t intensity = frame_buf[offset + 2U];

        /* ④ 点级滤波：盲区/超量程/弱回波 */
        if (distance < LDR_STL_MIN_DISTANCE_MM ||
            distance > LDR_STL_MAX_DISTANCE_MM) {
            continue;
        }
        if (intensity < LDR_STL_MIN_INTENSITY) {
            continue;
        }

#if LDR_STL_BODY_MASK_ENABLE
        /* ④b 车体自身：雷达相对车体固定，车体永远落在同一段角度上。
         * 车体把那个方向的激光挡住了，屏蔽这段不会丢真实障碍物。 */
        if (distance <= LDR_STL_BODY_MASK_MAX_MM &&
            angle_in_sector(angle, LDR_STL_BODY_MASK_START_DEG,
                            LDR_STL_BODY_MASK_END_DEG)) {
            continue;
        }
#endif

        collect_point(point_for_angle(angle, distance));
    }

    /* 本帧有效：刷新统计和超时基准 */
    valid_frames++;
    last_frame_tick = now;
    receive_active = true;
    return true;
}

/* 校验 6 字节状态包（0x54 0x1F + 3 字节状态 + CRC8）。
 * 实物雷达约每 500 ms 夹带一个，不参与测距，也不刷新帧超时。 */
static bool parse_status_frame(void)
{
    return (frame_buf[2] == 0U) && (frame_buf[3] == 0U) &&
           (frame_buf[4] == 0U) &&
           (ldr_crc8(frame_buf, LDR_STL_STATUS_FRAME_LEN - 1U) ==
            frame_buf[LDR_STL_STATUS_FRAME_LEN - 1U]);
}

/* 组帧状态机：逐字节喂入，拼出完整帧后交给解析函数。
 * 帧格式：0x54 0x2C + 44 字节数据 + CRC8 = 47 字节。
 * 帧头第二字节为 0x1F 时是 6 字节状态包。 */
static void feed_byte(uint8_t byte, uint32_t now)
{
    /* 等待帧头 */
    if (frame_index == 0U) {
        if (byte == LDR_STL_HEADER) {
            frame_buf[0] = byte;
            frame_index = 1U;
        }
        return;
    }

    /* 核对帧头第二字节 */
    if (frame_index == 1U) {
        if (byte == LDR_STL_VER_LEN) {
            frame_buf[1] = byte;
            frame_length = LDR_STL_FRAME_LEN;
            frame_index = 2U;
        } else if (byte == 0x1FU) {
            frame_buf[1] = byte;
            frame_length = LDR_STL_STATUS_FRAME_LEN;
            frame_index = 2U;
        } else {
            /* 帧头错乱：如果这个字节本身是 0x54，就把它当作新的帧头 */
            if (receive_active) bad_frames++;
            if (byte == LDR_STL_HEADER) {
                frame_buf[0] = byte;
                frame_index = 1U;
            } else {
                frame_index = 0U;
            }
        }
        return;
    }

    /* 收数据字节，直到凑满预期长度 */
    frame_buf[frame_index++] = byte;
    if (frame_index < frame_length) return;

    /* 收满一帧 */
    if (frame_length == LDR_STL_STATUS_FRAME_LEN) {
        if (!parse_status_frame()) bad_frames++;
    } else {
        if (!parse_measure_frame(now)) bad_frames++;
    }

    frame_index = 0U; /* 回到等待帧头状态 */
}

/* ============================ 对外接口 ============================ */

void ldrobot_stl_init(void)
{
    /* 清空统计与缓冲区 */
    rx_head = 0U;
    rx_tail = 0U;
    rx_dropped = 0U;
    observed_dropped = 0U;
    rx_error_pending = false;

    valid_frames = 0U;
    bad_frames = 0U;

    last_frame_tick = 0U;
    last_scan_tick = 0U;
    has_completed_scan = false;

    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.age_ms = UINT32_MAX;

    /* 复位解析/分圈/聚类状态（不清缓冲区） */
    invalidate_scan(false);

    /* 启动 USART6 中断接收；之后每次回调都会重新挂起接收 */
    (void)HAL_UARTEx_ReceiveToIdle_IT(&huart6, rx_chunk, LDR_STL_RX_CHUNK_SIZE);
}

void ldrobot_stl_process(void)
{
    uint32_t now = HAL_GetTick();

    /* 先做 500 ms 超时检查：雷达掉线时及时让结果失效 */
    expire_scan(now);

    /* 串口错误：丢弃残留字节，重新同步 */
    if (rx_error_pending) {
        rx_error_pending = false;
        observed_dropped = rx_dropped;
        invalidate_scan(true);
    }

    /* 中断侧发生过缓冲区溢出：帧边界已丢失，作废本圈并清空缓冲 */
    if (rx_dropped != observed_dropped) {
        observed_dropped = rx_dropped;
        invalidate_scan(true);
    }

    /* 把缓冲区中现有的字节全部解析完（不等待新数据） */
    uint8_t byte;
    while (ring_pop(&byte)) {
        feed_byte(byte, now);
    }

    /* 解析耗时期间中断可能又丢了字节，退出前再确认一次 */
    if (rx_dropped != observed_dropped) {
        observed_dropped = rx_dropped;
        invalidate_scan(true);
    }
}

void ldrobot_stl_rx_event_callback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart != &huart6) return;

    if (Size > LDR_STL_RX_CHUNK_SIZE) Size = LDR_STL_RX_CHUNK_SIZE;

    /* 单生产者（中断）写：先取一次读指针，满了就丢字节并计数 */
    uint16_t head = rx_head;
    uint16_t tail = rx_tail;

    for (uint16_t i = 0U; i < Size; ++i) {
        uint16_t next = (uint16_t)((head + 1U) & (LDR_STL_RING_SIZE - 1U));
        if (next == tail) {
            rx_dropped++; /* 缓冲区满：丢弃本字节，主循环会作废本圈 */
            continue;
        }
        rx_ring[head] = rx_chunk[i];
        head = next;
    }
    rx_head = head;

    /* 重新挂起接收，否则只会收到第一块数据 */
    (void)HAL_UARTEx_ReceiveToIdle_IT(&huart6, rx_chunk, LDR_STL_RX_CHUNK_SIZE);
}

void ldrobot_stl_rx_error_callback(UART_HandleTypeDef *huart)
{
    if (huart != &huart6) return;

    /* 清除所有接收错误标志，避免 HAL 状态机卡死 */
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_PE | UART_FLAG_FE |
                                     UART_FLAG_NE | UART_FLAG_ORE);
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    /* 通知主循环：数据流已不可信 */
    rx_error_pending = true;

    /* 重新挂起接收 */
    (void)HAL_UARTEx_ReceiveToIdle_IT(&huart6, rx_chunk, LDR_STL_RX_CHUNK_SIZE);
}

bool ldrobot_stl_get_scan(ldr_stl_scan_t *out)
{
    if (out == NULL) return false;

    uint32_t now = HAL_GetTick();
    expire_scan(now);

    /* age_ms：完成过扫描用当前时间差，否则用 UINT32_MAX 表示未知 */
    snapshot.age_ms = has_completed_scan
                          ? (uint32_t)(now - last_scan_tick)
                          : UINT32_MAX;

    *out = snapshot;
    return snapshot.valid;
}

uint8_t ldrobot_stl_get_obstacles(ldr_stl_obstacle_t *out, uint8_t max_count)
{
    if (out == NULL || max_count == 0U) return 0U;

    expire_scan(HAL_GetTick());
    if (!snapshot.valid) return 0U;

    uint8_t count = snapshot.count;
    if (count > max_count) count = max_count;

    if (count > 0U) {
        memcpy(out, snapshot.obstacles,
               (size_t)count * sizeof(ldr_stl_obstacle_t));
    }
    return count;
}

uint8_t ldrobot_stl_get_confirmed_obstacles(ldr_stl_obstacle_t *out,
                                            uint8_t max_count)
{
    if (out == NULL || max_count == 0U) return 0U;

    /* 先做超时检查：雷达掉线时 invalidate_scan() 会清空确认结果 */
    expire_scan(HAL_GetTick());

    uint8_t count = confirmed_count;
    if (count > max_count) count = max_count;

    if (count > 0U) {
        memcpy(out, confirmed_obstacles,
               (size_t)count * sizeof(ldr_stl_obstacle_t));
    }
    return count;
}

void ldrobot_stl_reset_confirm(void)
{
    memset(confirm_tracks, 0, sizeof(confirm_tracks));
    confirmed_count = 0U;
}

bool ldrobot_stl_is_valid(void)
{
    expire_scan(HAL_GetTick());
    return snapshot.valid;
}

uint32_t ldrobot_stl_valid_frames(void)
{
    return valid_frames;
}

uint32_t ldrobot_stl_bad_frames(void)
{
    return bad_frames;
}
