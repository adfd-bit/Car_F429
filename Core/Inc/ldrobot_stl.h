#ifndef LDROBOT_STL_H
#define LDROBOT_STL_H

/**
 * @file    ldrobot_stl.h
 * @brief   乐动 STL-06N(STL-06NBJ) 激光雷达驱动：USART6 接收、解析、聚类、滤波
 *
 * 数据手册要点（1-STL-06N数据手册.md / 2-STL-06N开发手册.md）：
 *   - 串口 USART6：230400 bit/s，8 数据位，1 停止位，无校验，无流控
 *   - 雷达单向发送，上电转动稳定后自动输出，不需要发送任何指令
 *   - 每帧固定 47 字节：0x54 0x2C + 转速(2) + 起始角(2) + 12×[距离(2)+强度(1)]
 *     + 结束角(2) + 时间戳(2) + CRC8(1)
 *   - 角度单位 0.01°，0° 为正前方，顺时针增大，范围 0~360°
 *   - 角度/距离由帧首尾角线性插值得到（12 个点等分 11 段）
 *
 * 数据处理流程（全部在本模块内完成）：
 *   ① 中断收字节 → 软件环形缓冲区（ISR 只写 head，主循环只写 tail）
 *   ② 主循环组帧：找 0x54 0x2C 帧头、按 47 字节拼帧、CRC8 校验
 *   ③ 逐点插值角度，检测角度跨 0° 作为"一圈结束"的分界
 *   ④ 距离/强度有效性过滤（滤掉盲区、超量程和弱回波噪点）
 *   ⑤ 聚类：相邻两点角度差 < 3° 且欧氏距离 < 150 mm 视为同一个障碍物
 *   ⑥ 滤波：组内点数过少的孤立噪点丢弃；用组内坐标平均值作为障碍物中心
 *   ⑦ 可选时域滤波：同一障碍物跨圈做指数平滑，抑制坐标抖动
 *   ⑧ 多圈确认：同一位置连续/累计命中 ≥ LDR_STL_CONFIRM_HITS 圈才认定为真障碍物
 *   ⑨ 一圈结束后按距离由近到远排序，写入对外快照（数组）
 *
 * 坐标系定义（与数据手册一致）：
 *   - 原点：默认是雷达旋转中心（可通过 LDR_STL_MOUNT_* 改成小车中心）
 *   - X 轴：向右为正
 *   - Y 轴：向前为正
 *   - angle_deg：相对车头方向的方位角，0°=正前，顺时针增大，范围 [0,360)
 *
 * ---------------- main.c 中的调用方法（共 4 处） ----------------
 *   1) 初始化（MX_USART6_UART_Init() 之后）：
 *          ldrobot_stl_init();
 *   2) 主循环（while(1) 开头，越频繁越好）：
 *          ldrobot_stl_process();
 *   3) 串口接收完成回调 HAL_UARTEx_RxEventCallback() 中：
 *          else if (huart == &huart6) ldrobot_stl_rx_event_callback(huart, Size);
 *   4) 串口错误回调 HAL_UART_ErrorCallback() 中：
 *          if (huart == &huart6) ldrobot_stl_rx_error_callback(huart);
 *
 *   取结果（任选一种）：
 *          // 方式一：取整圈快照。结构体较大，建议加 static，避免占用主栈
 *          static ldr_stl_scan_t scan;
 *          if (ldrobot_stl_get_scan(&scan) && scan.valid) {
 *              for (uint8_t i = 0; i < scan.count; ++i) {
 *                  // scan.obstacles[i].x_mm / y_mm / angle_deg / range_mm
 *              }
 *          }
 *
 *          // 方式二：只取坐标数组
 *          ldr_stl_obstacle_t list[8];
 *          uint8_t n = ldrobot_stl_get_obstacles(list, 8);
 *          for (uint8_t i = 0; i < n; ++i) {
 *              // list[i].x_mm, list[i].y_mm
 *          }
 *
 *          // 方式三：取多圈确认后的障碍物（路径规划推荐用这个）
 *          static ldr_stl_obstacle_t confirmed[LDR_STL_MAX_OBSTACLES];
 *          uint8_t m = ldrobot_stl_get_confirmed_obstacles(
 *                          confirmed, LDR_STL_MAX_OBSTACLES);
 *          for (uint8_t i = 0; i < m; ++i) {
 *              // confirmed[i].x_mm / y_mm, confirmed[i].hits
 *          }
 */

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* ========================= 1. 协议与缓冲区参数 ========================= */

#define LDR_STL_FRAME_LEN          47U  /* 测距帧固定长度：47 字节 */
#define LDR_STL_POINTS_PER_FRAME   12U  /* 每帧测距点数 */
#define LDR_STL_STATUS_FRAME_LEN   6U   /* 雷达夹带的 6 字节状态包：0x54 0x1F ... */
#define LDR_STL_HEADER             0x54U
#define LDR_STL_VER_LEN            0x2CU

#define LDR_STL_RX_CHUNK_SIZE      64U   /* HAL 每次中断接收的临时块大小（字节） */
#define LDR_STL_RING_SIZE          4096U /* 软件环形缓冲区大小，必须是 2 的幂 */
                                         /* 230400 波特率下约 200 ms 的数据量 */

#if ((LDR_STL_RING_SIZE & (LDR_STL_RING_SIZE - 1U)) != 0U)
#error "LDR_STL_RING_SIZE 必须是 2 的幂"
#endif

/* ========================= 2. 识别与滤波参数 ========================= */

/* 测距有效范围(mm)：超出的点按无效/噪点丢弃。
 *
 * MAX 从手册标称的 12000 收到 3500 —— 按比赛场地定的：
 *   场地 2400×2400，5×5 栅格最远一格的格心在车坐标 (1900,1900)，
 *   距原点 2687mm，加 320mm 标记半径 = 3007mm。
 *   也就是说超过 3 米的点永远标不进地图，留着只会占满
 *   LDR_STL_MAX_OBSTACLES(24) 的名额、把场地里的真实障碍物挤掉。
 * 实测：收之前 n 恒等于 24（满员，全是 3~5.6m 的实验室墙），收之后只剩场地内的。
 * 换场地或要测更远时改回 12000 即可。 */
#define LDR_STL_MIN_DISTANCE_MM    30U
#define LDR_STL_MAX_DISTANCE_MM    3500U

/* 强度下限：0 表示不过滤；现场噪声大可改成 10~20 试试 */
#define LDR_STL_MIN_INTENSITY      0U

/* ==================== 2.1 车体自身遮挡（扇区屏蔽） ==================== */
/* 车体结构件不会被现有滤波丢掉：它稳定、每圈都在、点数也够，必然通过聚类和
 * 多圈确认，最后被 PathPlan_UpdateFromRadar() 当成真障碍物投影进地图。
 *
 * 屏蔽办法：在雷达自己的坐标系里挖掉一段角度扇区（0°=车头，顺时针为正）。
 * 雷达相对车体固定安装，所以车体永远落在同一段角度上，跟车头朝哪无关；
 * 而且车体本来就把那个方向的激光挡住了，屏蔽掉它不会丢失任何真实障碍物。
 *
 * 怎么定这两个角（先保持 ENABLE = 0 跑一轮）：
 *   1) 把 ldrobot_stl_get_confirmed_obstacles() 每条的 angle_deg / range_mm 打出来；
 *   2) 离车最近、angle_deg 每圈几乎不变的那几条就是车体；
 *   3) 取它们角度的最小值往外放 5° 作 START，最大值放 5° 作 END；
 *   4) 把 ENABLE 改成 1。
 * 角度可以跨 0°（例如 START=350, END=20），angle_in_sector() 已处理回绕。
 * MAX_MM 是第二道保险：只屏蔽这么近的点。 */
#define LDR_STL_BODY_MASK_ENABLE    1
/* 标定依据（雷达 10Hz 稳定，CONFIRM_HITS=1）：
 *
 * ⚠ 注意：对外输出的 angle_deg / range_mm 是换算到"小车中心"后的值
 *   （point_for_angle() 加了 RIGHT=106 / FORWARD=106 的安装偏移），
 *   而这里的屏蔽用的是雷达自己的原始角度。标定时必须先反算回去：
 *       x_radar = r*sin(a) - 106
 *       y_radar = r*cos(a) - 106
 *       a_radar = atan2(x_radar, y_radar)
 *
 * 实测两簇车体回波（反算到雷达系）：
 *   屏蔽前 a=64.6°/r=158mm/176 点 → 雷达系 r=53.0mm  a=136.1°
 *   屏蔽后 a=71.8°/r=128mm/ 87 点 → 雷达系 r=67.8mm  a=166.7°
 * 176 点 × 0.72°/点 ≈ 127° 跨度，以 136.1° 为中心 → 约 72.7°~199.5°。
 * 0~135° 屏蔽掉之后剩 135~199.5°（64.5° ≈ 90 点），实测剩 87 点，吻合。
 * 另有 10 点的小簇在雷达系约 61°。所以车体大致占 57°~200°。
 *
 * 取 50°~205° 两端各留几度余量。车体把这段激光本来就挡住了，
 * 屏蔽它不损失任何真实障碍物。
 * 如果改完还有 ≤600mm 的目标冒出来，把 START 往小、END 往大继续放。 */
#define LDR_STL_BODY_MASK_START_DEG 50.0f
#define LDR_STL_BODY_MASK_END_DEG   205.0f
#define LDR_STL_BODY_MASK_MAX_MM    600U   /* 只屏蔽这个距离以内的点 */

/* 聚类条件：相邻两点同时满足下面两条才认为属于同一个障碍物 */
#define LDR_STL_GROUP_ANGLE_GAP_DEG    3.0f    /* 角度间隔 < 3° */
#define LDR_STL_GROUP_DISTANCE_MM      150.0f  /* 两点直线距离 < 150 mm */

/* 一个障碍物至少包含的原始点数：少于该值视为孤立噪点丢弃。
 *
 * 比赛用的黑色障碍物是 φ50×100mm 的细圆柱，张角很小：
 *   距离 0.5m → 5.72° | 1.0m → 2.86° | 1.5m → 1.91° | 2.0m → 1.43° | 2.4m → 1.19°
 * 10Hz 时角分辨率 0.72°/点，2.0m 处只有 2 个点 —— 阈值设 3 的话，
 * 1.4m 以外的黑色障碍物全都会被当噪点丢掉，等于看不见。
 *
 * 改成 2 之后：10Hz 可见到约 2.0m，6Hz（0.43°/点）可见到约 2.8m。
 * 代价是噪点变多，靠 LDR_STL_CONFIRM_MAX_MISSES 的滞回兜底。 */
#define LDR_STL_MIN_GROUP_POINTS   2U

/* 单次扫描最多输出多少个障碍物（按距离由近到远保留） */
#define LDR_STL_MAX_OBSTACLES      24U

/* 超时：超过这么久没有新帧/新一圈，认为雷达掉线，结果立即失效。
 *
 * SCAN_TIMEOUT 必须大于"转一圈的时间"，否则快照会在下一圈到来之前就作废。
 *   - 雷达正常 10Hz：一圈 100ms，500ms 绰绰有余；
 *   - 现在实测只有 ~1Hz：一圈约 1000ms，500ms 会导致每圈中间有半秒是无效的。
 * 等转速恢复正常后，这个值可以改回 500。 */
#define LDR_STL_SCAN_TIMEOUT_MS    1500U
#define LDR_STL_FRAME_TIMEOUT_MS   500U

/* ===================== 3. 雷达安装位置（相对小车中心） ===================== */

/* 默认 0 = 坐标原点就是雷达旋转中心，且雷达 0° 与车头同向。
 * 雷达不在小车中心时，把实测偏移填进来，所有障碍物坐标会自动换算到小车中心
 * （也就是 OPS9 定位的那个点），路径规划直接可用。
 *
 * 符号约定（单位 mm / 度）：
 *   RIGHT   ：雷达旋转中心在小车中心右侧为正，左侧为负
 *   FORWARD ：雷达旋转中心在小车中心前方为正，后方为负
 *   HEADING ：雷达 0° 方向相对车头顺时针偏转为正，逆时针为负
 *
 * 例：雷达装在小车中心左侧 80 mm、前方 120 mm，且整体顺时针偏 3°：
 *     RIGHT = -80.0f, FORWARD = 120.0f, HEADING = 3.0f
 *
 * 补偿在 point_for_angle() 里逐点完成，之后的聚类、时域滤波、
 * 多圈确认、路径规划拿到的都是相对小车中心的坐标。 */
#define LDR_STL_MOUNT_RIGHT_MM     106    /* 雷达旋转中心在 OPS9 参考点右侧 106 mm */
#define LDR_STL_MOUNT_FORWARD_MM   106    /* 雷达旋转中心在 OPS9 参考点前方 106 mm */
#define LDR_STL_MOUNT_HEADING_DEG  0.0f   /* 顺时针为正，逆时针为负 */

/* 倒装补偿：雷达上下翻转后，旋转方向会反向（顺时针变逆时针），
 * 此时"雷达报告角"和"真实方位角"是镜像关系，不是简单加 180°：
 *     θ真实 = LDR_STL_MOUNT_MIRROR_DEG − θ报告
 *     0   = 正装，不补偿
 *     360 = 倒装，翻转轴沿车头方向（θ → −θ，最常见的倒装方式）
 *     180 = 倒装，翻转轴沿车宽方向（雷达前端朝后）
 *
 * 定 K 的依据（实测车体回波在报告角 57°~200°，雷达装在车的右前方，
 * 所以车体真实方位应在左后方 160°~300°）：
 *     360 − 200 = 160 ✓      360 − 57 = 303 ≈ 300 ✓
 * 建议再用一个已知方向的障碍物验证一次：
 *     真实 315°（左前 1m）→ 补偿后应读到 a≈315，补偿前是 a≈45。
 *
 * ⚠ 只影响坐标换算，不影响车体屏蔽：屏蔽用的是 point_for_angle()
 *   之前的雷达原始角度，标定值 50~205° 与正装/倒装无关，不要动。 */
#define LDR_STL_MOUNT_MIRROR_DEG   360.0f

/* ========================= 4. 时域滤波参数 ========================= */

/* 1 = 开启跨圈指数平滑（推荐，坐标更稳）；0 = 关闭，输出每圈原始结果 */
#define LDR_STL_ENABLE_TEMPORAL_FILTER  1

/* 新结果权重 α：0~1。越大越跟手、越小越平滑；
 * 1.0 等价于不滤波，0.6 大约有 2~3 圈的平滑记忆 */
#define LDR_STL_TEMPORAL_ALPHA          0.6f

/* 跨圈匹配门限：只有当新障碍物与上一圈某个障碍物
 * 方位角差 ≤ 5° 且距离差 ≤ 300 mm 时才认为是同一个目标并做平滑 */
#define LDR_STL_TRACK_ANGLE_GAP_DEG     5.0f
#define LDR_STL_TRACK_RANGE_GAP_MM      300.0f

/* ===================== 4.1 多圈确认（持久化）参数 ===================== */

/* 同一个位置累计被检测到多少圈之后，才通过
 * ldrobot_stl_get_confirmed_obstacles() 输出。
 * 设为 1 表示新目标第一圈就确认（仍保留 MAX_MISSES 的丢失滞回）。
 * 10 Hz 扫描时，3 圈约 0.3 秒即可确认一个稳定目标。 */
#define LDR_STL_CONFIRM_HITS            1U

/* 确认跟踪表容量：允许"候选目标"和"已确认目标"同时存在。
 * 比 LDR_STL_MAX_OBSTACLES 略大，避免候选目标把已确认目标挤掉。 */
#define LDR_STL_MAX_TRACKS              32U

/* 确认层跨圈配对门限（与新目标/旧跟踪的匹配条件） */
#define LDR_STL_CONFIRM_ANGLE_GAP_DEG   5.0f
#define LDR_STL_CONFIRM_RANGE_GAP_MM    300.0f

/* 已经确认的障碍物允许连续丢失多少圈后才删除（滞回）。
 * 防止某一圈偶尔漏检就瞬间把障碍物从路径规划里移除。
 * 10 Hz 时 3 圈约 0.3 秒。 */
#define LDR_STL_CONFIRM_MAX_MISSES      3U

/* ============================ 5. 数据结构 ============================ */

/* 一个障碍物（聚类后的一组点的中心） */
typedef struct {
    int32_t  x_mm;       /* 小车坐标系 X：向右为正，单位 mm */
    int32_t  y_mm;       /* 小车坐标系 Y：向前为正，单位 mm */
    uint32_t range_mm;   /* 到坐标原点的直线距离，单位 mm */
    float    angle_deg;  /* 相对车头的方位角，0°=正前，顺时针增大，[0,360) */
    uint16_t points;     /* 该障碍物由多少个原始测距点聚类而成（可作置信度参考） */
    uint8_t  hits;       /* 多圈确认：已连续/累计命中的圈数，饱和于 LDR_STL_CONFIRM_HITS */
} ldr_stl_obstacle_t;

/* 一次完整扫描（雷达转一圈）的结果快照 */
typedef struct {
    bool     valid;      /* 本快照是否有效（完成过至少一圈且未超时） */
    uint32_t scan_seq;   /* 完整扫描序号，每发布一圈 +1 */
    uint32_t age_ms;     /* 距本圈发布的时间(ms)；从未完成扫描时为 UINT32_MAX */
    uint8_t  count;      /* obstacles 数组中有效项的个数 */
    ldr_stl_obstacle_t obstacles[LDR_STL_MAX_OBSTACLES]; /* 按距离由近到远排列 */
} ldr_stl_scan_t;

/* ============================ 6. 对外接口 ============================ */

/**
 * @brief 初始化雷达模块并启动 USART6 中断接收
 * @note  必须在 MX_USART6_UART_Init() 之后调用一次。
 *        模块内部会按 230400 8N1 的 USART6 配置直接启动 HAL 接收。
 */
void ldrobot_stl_init(void);

/**
 * @brief 雷达数据处理任务，必须在主循环中反复调用
 * @note  只做"从软件缓冲区取字节 + 解析"，不会阻塞等待串口。
 *        调用越频繁越好；长时间不调用会导致软件缓冲区溢出（溢出后本圈作废并自动重新同步）。
 */
void ldrobot_stl_process(void);

/**
 * @brief 串口接收事件钩子（在 HAL_UARTEx_RxEventCallback 中调用）
 * @param huart 触发回调的串口句柄
 * @param Size  本次收到的字节数
 * @note  只处理 huart6；内部会把数据压入软件缓冲区并重新启动接收。
 *        在 main.c 的 HAL_UARTEx_RxEventCallback() 里加一行：
 *        else if (huart == &huart6) ldrobot_stl_rx_event_callback(huart, Size);
 */
void ldrobot_stl_rx_event_callback(UART_HandleTypeDef *huart, uint16_t Size);

/**
 * @brief 串口错误钩子（在 HAL_UART_ErrorCallback 中调用）
 * @param huart 触发回调的串口句柄
 * @note  只处理 huart6；内部会清错误标志、作废当前扫描并重启接收。
 *        在 main.c 的 HAL_UART_ErrorCallback() 里加一行：
 *        if (huart == &huart6) ldrobot_stl_rx_error_callback(huart);
 */
void ldrobot_stl_rx_error_callback(UART_HandleTypeDef *huart);

/**
 * @brief 取最近一圈的完整扫描结果（整份拷贝）
 * @param out 输出缓冲区，不能为 NULL
 * @retval true  快照有效（雷达在线且最近一圈已完成）
 * @retval false 还没有完成过完整一圈，或雷达超过 500 ms 没有数据
 * @note  返回 false 时 out 中的 count 为 0，其余内容无意义。
 */
bool ldrobot_stl_get_scan(ldr_stl_scan_t *out);

/**
 * @brief 只取障碍物坐标数组（常用简化接口）
 * @param out       调用方提供的数组
 * @param max_count 数组容量
 * @return 实际拷贝的障碍物个数（0 表示当前无有效结果）
 * @note  结果按距离由近到远排列，最多拷贝 min(snapshot.count, max_count) 个。
 */
uint8_t ldrobot_stl_get_obstacles(ldr_stl_obstacle_t *out, uint8_t max_count);

/**
 * @brief 取"多圈确认"后的障碍物数组（供路径规划调用）
 * @param out       调用方数组
 * @param max_count 数组容量
 * @return 实际拷贝个数（0 表示没有达到确认门限的障碍物）
 * @note  同一个位置必须累计命中 LDR_STL_CONFIRM_HITS 圈才会出现在结果里，
 *        可有效滤掉单圈噪点/反光。结果按距离由近到远排列。
 *        候选目标丢一圈命中数 -1；已确认目标允许连续丢失
 *        LDR_STL_CONFIRM_MAX_MISSES 圈（滞回），超过后才移除。
 *        雷达超时/数据流作废时，确认结果会一起清空。
 */
uint8_t ldrobot_stl_get_confirmed_obstacles(ldr_stl_obstacle_t *out,
                                            uint8_t max_count);

/**
 * @brief 清空多圈确认状态（重新开始一轮"扫描-确认"）
 * @note  开始一次全新的 13 秒扫描前调用，避免上一次的命中记录残留。
 *        不会影响串口接收和每圈扫描结果。
 */
void ldrobot_stl_reset_confirm(void);

/**
 * @brief 当前是否存在有效的完整扫描（内部会先做超时检查）
 */
bool ldrobot_stl_is_valid(void);

/* 诊断计数：累计校验通过 / 失败的帧数，可用于判断接线和波特率是否正确 */
uint32_t ldrobot_stl_valid_frames(void);
uint32_t ldrobot_stl_bad_frames(void);

/* ==================== 串口控制指令（开发手册 2.2 节） ====================
 * 指令帧：0x54 | Mode | 0x04 | Data[4] | CRC8，共 8 字节。
 * 雷达每次都会回一条同格式的 ACK（混在测距数据流里）。
 * 转速范围只有 2160~4680 度/秒（6~13Hz），设不了更慢的转速。 */

/* 启动电机（Mode=0xA0）。上电默认就是转的，一般不需要调。 */
bool ldrobot_stl_cmd_start(void);

/* 停止电机（Mode=0xA1）。停转后不再输出测距帧，500ms 后扫描结果自动失效。 */
bool ldrobot_stl_cmd_stop(void);

/* 设置目标转速（Mode=0xA2），单位度/秒。
 * 合法范围 2160~4680（6~13Hz）；超出范围直接返回 false，雷达也不响应。
 * 掉电丢失，重新上电回到默认 10Hz。 */
bool ldrobot_stl_set_speed(uint16_t deg_per_s);

/* 查询目标转速（Mode=0xA3）。ACK 目前不单独解析。 */
bool ldrobot_stl_query_speed(void);

#define LDR_STL_SPEED_6HZ_DPS   2160U  /* 6Hz  */
#define LDR_STL_SPEED_10HZ_DPS  3600U  /* 10Hz，上电默认 */
#define LDR_STL_SPEED_13HZ_DPS  4680U  /* 13Hz */

/* 开机时下发的目标转速（main.c 里调用 ldrobot_stl_set_speed()）。
 *
 * 测距频率固定 5000 点/秒，所以转得越慢，角分辨率越细：
 *   10Hz → 500 点/圈 → 0.72°/点（数据手册标称值）
 *    6Hz → 833 点/圈 → 0.43°/点   ← 角分辨率提高 1.67 倍
 *
 * 原地静止扫描时不关心刷新率，用 6Hz 能让 φ50mm 的小障碍物多出近一倍的点，
 * 更容易通过 MIN_GROUP_POINTS 的聚类门限。
 * 等要跑动避障、需要更快刷新时，改回 LDR_STL_SPEED_10HZ_DPS。 */
#define LDR_STL_SCAN_SPEED_DPS  LDR_STL_SPEED_6HZ_DPS

#endif /* LDROBOT_STL_H */
