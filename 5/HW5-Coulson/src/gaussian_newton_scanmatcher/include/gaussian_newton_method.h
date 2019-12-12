#ifndef GAUSSIAN_NEWTON_METHOD_H
#define GAUSSIAN_NEWTON_METHOD_H

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "map.h"
#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>

//用激光雷达数据创建势场．
map_t* CreateMapFromLaserPoints(Eigen::Vector3d map_origin_pt,
    std::vector<Eigen::Vector2d> laser_pts,
    double resolution);

//用高斯牛顿的方法来进行优化
void GaussianNewtonOptimization(map_t* map, Eigen::Vector3d& init_pose, std::vector<Eigen::Vector2d>& laser_pts);

#endif // GAUSSIAN_NEWTON_METHOD_H
