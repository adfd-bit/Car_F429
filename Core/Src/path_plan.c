/*
 * path_plan.c
 *
 *  Created on: 2026年7月20日
 *      Author: twyyd
 */
#include "path_plan.h"
#include "OPS9.h"
#include "ldrobot_stl.h"

#include <math.h>
#include <string.h>

// 0:可走
// 1:障碍

uint8_t map[MAP_SIZE][MAP_SIZE] = {{0, 0, 0, 0, 0},
                                   {0, 1, 0, 1, 0},
                                   {0, 0, 0, 0, 0},
                                   {0, 1, 0, 1, 0},
                                   {0, 0, 0, 0, 0}};

/* 激光雷达动态障碍物层：与静态 map 独立，
 * 每次更新先清空再按最新确认结果重建，不会污染固定地图。 */
static uint8_t radar_map[MAP_SIZE][MAP_SIZE];
Node nodes[MAP_SIZE][MAP_SIZE];
int path_length;
PathPoint path[MAP_SIZE * MAP_SIZE];
int dir[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
int Manhattan(int x1, int y1, int x2, int y2) {
  return abs(x1 - x2) + abs(y1 - y2);
}
void InitNodes() {
  for (int i = 0; i < MAP_SIZE; i++) {
    for (int j = 0; j < MAP_SIZE; j++) {

      nodes[i][j].x = i;
      nodes[i][j].y = j;

      nodes[i][j].real_x = 0.0;
      nodes[i][j].real_y = 0.0;
      nodes[i][j].real_r = 0.0;

      nodes[i][j].g = INF;
      nodes[i][j].h = 0;
      nodes[i][j].f = INF;

      nodes[i][j].parent_x = -1;
      nodes[i][j].parent_y = -1;

      nodes[i][j].open = false;
      nodes[i][j].close = false;
    }
  }
}
void Initreal() {
  nodes[0][0].real_x = 150;
  nodes[0][0].real_y = 150;

  nodes[MAP_MID][0].real_x = 1050;
  nodes[MAP_MID][0].real_y = 200;

  nodes[MAP_END][0].real_x = 1900;
  nodes[MAP_END][0].real_y = 200;

  nodes[0][MAP_MID].real_x = 200;
  nodes[0][MAP_MID].real_y = 1050;

  nodes[0][MAP_END].real_x = 200;
  nodes[0][MAP_END].real_y = 1900;

  nodes[MAP_MID][MAP_MID].real_x = 1050;
  nodes[MAP_MID][MAP_MID].real_y = 1030;

  nodes[MAP_MID][MAP_END].real_x = 1050;
  nodes[MAP_MID][MAP_END].real_y = 1900;

  nodes[MAP_END][MAP_MID].real_x = 1940;
  nodes[MAP_END][MAP_MID].real_y = 1050;

  nodes[MAP_END][MAP_END].real_x = 1900;
  nodes[MAP_END][MAP_END].real_y = 1900;
}
int IsValid(int x, int y) {
  if (x < 0 || x >= MAP_SIZE)
    return 0;
  if (y < 0 || y >= MAP_SIZE)
    return 0;
  if (map[x][y] == 1)
    return 0;
  if (radar_map[x][y] == 1) {
    return 0;
  }

  return 1;
}

Node *GetMinNode() {
  Node *min = NULL;
  Node *n;
  for (int i = 0; i < MAP_SIZE; i++) {
    for (int j = 0; j < MAP_SIZE; j++) {
      n = &nodes[i][j];
      if (n->open && !n->close) {
        if (min == NULL || n->f < min->f) {
          min = n;
        }
      }
    }
  }
  return min;
}
int AStar(int start_x, int start_y, int goal_x, int goal_y) {
  InitNodes();
  Initreal();
  Node *current;
  Node *next;
  Node *start = &nodes[start_x][start_y];
  start->g = 0;
  start->h = Manhattan(start_x, start_y, goal_x, goal_y);
  start->f = start->g + start->h;
  start->open = true;
  while (1) {
    current = GetMinNode();
    if (current == NULL) { // 无有效路径
      return 0;
    }
    if (current->x == goal_x && current->y == goal_y) { // 到达目标
      return 1;
    }
    current->close = true;
    for (int i = 0; i < 4; i++) {
      int nx = current->x + dir[i][0];
      int ny = current->y + dir[i][1];
      if (!IsValid(nx, ny))
        continue;
      next = &nodes[nx][ny];
      if (next->close)
        continue;
      int new_g = current->g + 1;
      if (!next->open || new_g < next->g) {
        next->g = new_g;
        next->h = Manhattan(nx, ny, goal_x, goal_y);
        next->f = next->g + next->h;
        next->parent_x = current->x;
        next->parent_y = current->y;
        next->open = true;
      }
    }
  }
}
void GetPath(int x, int y) {
  int py;
  int px;
  path_length = 0;
  while (1) {
    path[path_length].x = x;
    path[path_length].y = y;
    path_length++;
    if (nodes[x][y].parent_x == -1)
      break;
    px = nodes[x][y].parent_x;
    py = nodes[x][y].parent_y;
    x = px;
    y = py;
  }
}
void ReversePath() {
  for (int i = 0; i < path_length / 2; i++) {
    PathPoint temp = path[i];
    path[i] = path[path_length - 1 - i];
    path[path_length - 1 - i] = temp;
  }
}
bool Find_path(int start_x, int start_y, int goal_x, int goal_y) {
#if PATH_USE_RADAR_OBSTACLES
  /* 规划前用最新"多圈确认"结果刷新动态障碍层。
   * 雷达无效时保留上一次扫到的结果，不会把障碍物清空。 */
  (void)PathPlan_UpdateFromRadar();
#endif
  if (AStar(start_x, start_y, goal_x, goal_y)) {
    GetPath(goal_x, goal_y);
    ReversePath();
    return true;
  } else {
    return false;
  }
}
/* ==================== 激光雷达动态障碍物层 ==================== */

void PathPlan_ClearRadarObstacles(void) {
  memset(radar_map, 0, sizeof(radar_map));
}

/* 把一个场地坐标(mm)换算成栅格下标并标记。
 * 只标记距栅格中心 PATH_RADAR_MARK_RADIUS_MM 以内的点，
 * 超出 5×5 地图范围的点直接忽略。 */
bool PathPlan_MarkObstacleMm(float x_mm, float y_mm) {
  int i = (int)lroundf((x_mm - PATH_RADAR_CELL_ORIGIN_MM) /
                       PATH_RADAR_CELL_SIZE_MM);
  int j = (int)lroundf((y_mm - PATH_RADAR_CELL_ORIGIN_MM) /
                       PATH_RADAR_CELL_SIZE_MM);

  if (i < 0 || i >= MAP_SIZE || j < 0 || j >= MAP_SIZE) {
    return false;
  }

  float cx = PATH_RADAR_CELL_ORIGIN_MM + (float)i * PATH_RADAR_CELL_SIZE_MM;
  float cy = PATH_RADAR_CELL_ORIGIN_MM + (float)j * PATH_RADAR_CELL_SIZE_MM;
  float dx = x_mm - cx;
  float dy = y_mm - cy;

  if (dx * dx + dy * dy >
      PATH_RADAR_MARK_RADIUS_MM * PATH_RADAR_MARK_RADIUS_MM) {
    return false;
  }

  radar_map[i][j] = 1U;
  return true;
}

/* 用雷达"多圈确认"结果重建动态障碍层，返回本次标记的栅格数。
 *
 * 坐标换算：
 *   雷达输出 (x右, y前)，angle_deg 为相对车头的方位角（顺时针为正）；
 *   OPS9 实时位姿 OPS_X/OPS_Y(mm) 与 OPS_angle（0°=+Y，逆时针为正）。
 *   场地方向角 = angle_deg - OPS_angle
 *     fx = OPS_X + range * sin(场地方向角)
 *     fy = OPS_Y + range * cos(场地方向角)
 *
 * 注意：这里用实时 OPS_X/OPS_Y/OPS_angle，而不是 current_x/current_y——
 *       后者是"上一次到达的目标点"，只在到位时才更新，不能当实时位姿用。
 * 雷达结果本身有约 100 ms（10 Hz 一圈）延迟，对栅格级规划可以忽略。
 *
 * 雷达无效（未初始化/超时）时保留上一次的动态层：
 * 这样"先扫 13 秒，再导航"的流程中，导航途中没调用
 * ldrobot_stl_process() 也不会把已经扫到的障碍物清空。 */
uint8_t PathPlan_UpdateFromRadar(void) {
  static ldr_stl_obstacle_t list[LDR_STL_MAX_OBSTACLES];
  uint8_t marked = 0U;

  if (!ldrobot_stl_is_valid()) {
    return 0U;
  }

  PathPlan_ClearRadarObstacles();

  uint8_t count =
      ldrobot_stl_get_confirmed_obstacles(list, (uint8_t)LDR_STL_MAX_OBSTACLES);

  for (uint8_t k = 0U; k < count; ++k) {
    float rel = (list[k].angle_deg - OPS_angle) * (3.14159265f / 180.0f);
    float field_x = OPS_X + (float)list[k].range_mm * sinf(rel);
    float field_y = OPS_Y + (float)list[k].range_mm * cosf(rel);

    if (PathPlan_MarkObstacleMm(field_x, field_y)) {
      marked++;
    }
  }

  return marked;
}
