#include "gaussian_newton_method.h"
#include <cmath>
#include <map.h>

const double GN_PI = 3.1415926;

using namespace std;

//进行角度正则化．
double GN_NormalizationAngle(double angle)
{
    if (angle > GN_PI)
        angle -= 2 * GN_PI;
    else if (angle < -GN_PI)
        angle += 2 * GN_PI;

    return angle;
}

Eigen::Matrix3d GN_V2T(Eigen::Vector3d vec)
{
    Eigen::Matrix3d T;
    T << cos(vec(2)), -sin(vec(2)), vec(0),
        sin(vec(2)), cos(vec(2)), vec(1),
        0, 0, 1;

    return T;
}

//对某一个点进行转换．
Eigen::Vector2d GN_TransPoint(Eigen::Vector2d pt, Eigen::Matrix3d T)
{
    Eigen::Vector3d tmp_pt(pt(0), pt(1), 1);
    tmp_pt = T * tmp_pt;
    return Eigen::Vector2d(tmp_pt(0), tmp_pt(1));
}

//用激光雷达数据创建势场．
map_t* CreateMapFromLaserPoints(Eigen::Vector3d map_origin_pt,
    std::vector<Eigen::Vector2d> laser_pts,
    double resolution)
{
    map_t* map = map_alloc();

    map->origin_x = map_origin_pt(0);
    map->origin_y = map_origin_pt(1);
    map->resolution = resolution;

    //固定大小的地图，必要时可以扩大．
    map->size_x = 10000;
    map->size_y = 10000;

    map->cells = (map_cell_t*)malloc(sizeof(map_cell_t) * map->size_x * map->size_y);

    //高斯平滑的sigma－－固定死
    map->likelihood_sigma = 0.5;

    Eigen::Matrix3d Trans = GN_V2T(map_origin_pt);

    //设置障碍物
    for (int i = 0; i < laser_pts.size(); i++) {
        Eigen::Vector2d tmp_pt = GN_TransPoint(laser_pts[i], Trans);

        int cell_x, cell_y;
        cell_x = MAP_GXWX(map, tmp_pt(0));
        cell_y = MAP_GYWY(map, tmp_pt(1));

        map->cells[MAP_INDEX(map, cell_x, cell_y)].occ_state = CELL_STATUS_OCC;
    }

    //进行障碍物的膨胀--最大距离固定死．
    map_update_cspace(map, 0.5);

    return map;
}

/**
 * @brief InterpMapValueWithDerivatives
 * 在地图上的进行插值，得到coords处的势场值和对应的关于位置的梯度．
 * 返回值为Eigen::Vector3d ans
 * ans(0)表示势场值
 * ans(1:2)表示梯度
 * @param map
 * @param coords
 * @return
 */
Eigen::Vector3d InterpMapValueWithDerivatives(map_t* map, Eigen::Vector2d& coords)
{
    Eigen::Vector3d ans;
    //TODO
    // cout << "coords: " << coords.transpose() << endl;
    if (!MAP_VALID(map, coords[0], coords[1])) {
        return Eigen::Vector3d(0.0, 0.0, 0.0);
    }

    Eigen::Vector2i int_coords(coords.cast<int>());

    Eigen::Vector2d factors(coords - int_coords.cast<double>());
    // cout << "-------" << endl;
    // cout << "coords: " << coords.transpose() << endl;
    // cout << "int_coords: " << int_coords.transpose() << endl;
    // cout << "factors : " << factors.transpose() << endl;

    int sizeX = map->size_x;

    Eigen::Vector4d intensities;

    intensities[0] = map->cells[MAP_INDEX(map, int_coords[0], int_coords[1])].score;
    intensities[1] = map->cells[MAP_INDEX(map, int_coords[0] + 1, int_coords[1])].score;
    intensities[2] = map->cells[MAP_INDEX(map, int_coords[0], int_coords[1] + 1)].score;
    intensities[3] = map->cells[MAP_INDEX(map, int_coords[0] + 1, int_coords[1] + 1)].score;

    // cout << "intensities: " << intensities.transpose() << endl;

    double dx1 = intensities[0] - intensities[1];
    double dx2 = intensities[2] - intensities[3];

    double dy1 = intensities[0] - intensities[2];
    double dy2 = intensities[1] - intensities[3];

    double xFacInv = (1.0 - factors[0]);
    double yFacInv = (1.0 - factors[1]);

    ans = Eigen::Vector3d(
        ((intensities[0] * xFacInv + intensities[1] * factors[0]) * (yFacInv)) + ((intensities[2] * xFacInv + intensities[3] * factors[0]) * (factors[1])),
        -((dx1 * yFacInv) + (dx2 * factors[1])) / map->resolution,
        -((dy1 * xFacInv) + (dy2 * factors[0])) / map->resolution);

    //END OF TODO

    return ans;
}

/**
 * @brief ComputeCompleteHessianAndb
 * 计算H*dx = b中的H和b
 * @param map
 * @param now_pose
 * @param laser_pts
 * @param H
 * @param b
 */
void ComputeHessianAndb(map_t* macoordsp, Eigen::Vector3d now_pose,
    std::vector<Eigen::Vector2d>& laser_pts,
    Eigen::Matrix3d& H, Eigen::Vector3d& b)
{
    H = Eigen::Matrix3d::Zero();
    b = Eigen::Vector3d::Zero();

    //TODO

    Eigen::Matrix3d trans_now;
    double sin_angle = sin(now_pose[2]);
    double cos_angle = cos(now_pose[2]);
    trans_now << cos_angle, -sin_angle, now_pose[0],
        sin_angle, cos_angle, now_pose[1],
        0, 0, 1;

    for (size_t i = 0; i < laser_pts.size(); i++) {
        // 激光雷达坐标系下的激光点的坐标
        const Eigen::Vector2d& pt = laser_pts[i];
        Eigen::Vector3d pt_tmp(pt[0], pt[1], 1);

        // now_pose是当前待估计的激光雷达在世界坐标系下面的坐标
        // now此刻激光点云在世界坐标系下面的坐标
        Eigen::Vector3d pt_tmp2 = trans_now * pt_tmp;

        Eigen::Vector2d transformed_pt(pt_tmp2[0], pt_tmp2[1]);

        double cell_x_double, cell_y_double;
        cell_x_double = (transformed_pt[0] - macoordsp->origin_x) / macoordsp->resolution + double(macoordsp->size_x / 2);
        cell_y_double = (transformed_pt[1] - macoordsp->origin_y) / macoordsp->resolution + double(macoordsp->size_y / 2);
        Eigen::Vector2d coord(cell_x_double, cell_y_double);

        Eigen::Vector3d transformedPointData = InterpMapValueWithDerivatives(macoordsp, coord);

        double funVal = 1.0 - transformedPointData[0];
        // cout << "transformedPointData: " << transformedPointData.transpose() << endl;
        // cout << "funVal: " << funVal << endl;

        b[0] += transformedPointData[1] * funVal;
        b[1] += transformedPointData[2] * funVal;

        double rotDeriv = ((-sin_angle * pt.x() - cos_angle * pt.y()) * transformedPointData[1] + (cos_angle * pt.x() - sin_angle * pt.y()) * transformedPointData[2]);
        // cout << "rotDeriv: " << rotDeriv << endl;
        b[2] += rotDeriv * funVal;
        // cout << " H(0,0) " << H(0, 0) << " b[2] " << b[2] << endl;
        H(0, 0) += transformedPointData[1] * transformedPointData[1];
        H(1, 1) += transformedPointData[2] * transformedPointData[2];
        H(2, 2) += rotDeriv * rotDeriv;

        H(0, 1) += transformedPointData[1] * transformedPointData[2];
        H(0, 2) += transformedPointData[1] * rotDeriv;
        H(1, 2) += transformedPointData[2] * rotDeriv;
    }

    H(1, 0) = H(0, 1);
    H(2, 0) = H(0, 2);
    H(2, 1) = H(1, 2);
    //END OF TODO
}

/**
 * @brief GaussianNewtonOptimization
 * 进行高斯牛顿优化．
 * @param map
 * @param init_pose
 * @param laser_pts
 */
void GaussianNewtonOptimization(map_t* map, Eigen::Vector3d& init_pose, std::vector<Eigen::Vector2d>& laser_pts)
{
    int maxIteration = 30;
    Eigen::Vector3d now_pose = init_pose;
    Eigen::Matrix3d H;
    Eigen::Vector3d b;
    for (int i = 0; i < maxIteration; i++) {
        //TODO
        ComputeHessianAndb(map, now_pose, laser_pts, H, b);
        // std::cout << H(0, 0) << H(1, 1) << H(2, 2) << std::endl;
        if ((H(0, 0) != 0.0) && (H(1, 1) != 0.0)) {
            Eigen::Vector3d searchDir(H.inverse() * b);

            if (searchDir[2] > 0.2) {
                searchDir[2] = 0.2;
                std::cout << "SearchDir angle change too large\n";
            } else if (searchDir[2] < -0.2) {
                searchDir[2] = -0.2;
                std::cout << "SearchDir angle change too large\n";
            }

            // update
            now_pose += searchDir;
        }
        //END OF TODO
    }
    init_pose = now_pose;
}
