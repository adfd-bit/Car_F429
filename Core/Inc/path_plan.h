/*
 * path_plan.h
 *
 *  Created on: 2026年7月20日
 *      Author: twyyd
 */

#ifndef INC_PATH_PLAN_H_
#define INC_PATH_PLAN_H_

#include "stdbool.h"
#include "stdint.h"
#include "stdio.h"
#include "stdlib.h"
#include "usart.h"


#define MAP_SIZE 5
#define MAP_END (MAP_SIZE - 1)
#define MAP_MID (MAP_END / 2)
#define MAP_OBS (MAP_MID / 2)
#define INF 9999

typedef struct {
  int x;
  int y;

  float real_x;
  float real_y;
  float real_r;

  int g;
  int h;
  int f;

  int parent_x;
  int parent_y;

  bool open;
  bool close;
} Node;

typedef struct {
  uint8_t x;
  uint8_t y;

} PathPoint;

int Manhattan(int x1, int y1, int x2, int y2);
void InitNodes();
int IsValid(int x, int y);
void Initreal();
Node *GetMinNode();
int AStar(int start_x, int start_y, int goal_x, int goal_y);
void GetPath(int x, int y);
void ReversePath();
bool Find_path(int start_x, int start_y, int goal_x, int goal_y);

/* ==================== 激光雷达动态障碍物层 ====================
 * 雷达坐标系：原点=车体中心，X 向右，Y 向前，角度 0°=正前、顺时针。
 * 场地坐标系：OPS9 实时位姿 OPS_X/OPS_Y（mm）和 OPS_angle（度），
 *             0°=+Y，逆时针为正（-90°=+X）。
 * 换算：场地方向角 = 雷达方位角 - OPS_angle
 * 注意：不能用 current_x/current_y，它们只在到位时更新，不是实时位姿。
 *
 * 该层不修改原有静态 map，只在 IsValid() 里叠加判断；
 * 每次 PathPlan_UpdateFromRadar() 都会清空并按最新确认结果重建；
 * 雷达无效（未初始化/超时）时保留上一次结果，避免导航途中被清空。
 * 建议在 13 秒扫描结束后、小车静止且 OPS9 定位有效时调用一次。
 */
#define PATH_USE_RADAR_OBSTACLES 1       /* 1=Find_path 自动刷新雷达障碍层 */
#define PATH_RADAR_CELL_ORIGIN_MM 150.0f /* 栅格(0,0)中心对应的场地坐标(mm) */
#define PATH_RADAR_CELL_SIZE_MM 437.5f   /* 相邻栅格中心间距(mm) */
#define PATH_RADAR_MARK_RADIUS_MM 320.0f /* 障碍物距栅格中心小于该值才标记 */

void PathPlan_ClearRadarObstacles(void);
bool PathPlan_MarkObstacleMm(float x_mm, float y_mm);
uint8_t PathPlan_UpdateFromRadar(void);

#endif /* INC_PATH_PLAN_H_ */
