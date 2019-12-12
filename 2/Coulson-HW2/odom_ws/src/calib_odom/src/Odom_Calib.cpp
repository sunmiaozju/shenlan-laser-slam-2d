#include "../include/calib_odom/Odom_Calib.hpp"

//设置数据长度,即多少数据计算一次
void OdomCalib::Set_data_len(int len)
{
    data_len = len;
    A.conservativeResize(len * 3, 9);
    b.conservativeResize(len * 3);
    A.setZero();
    b.setZero();
}

/*
输入:里程计和激光数据

TODO:
构建最小二乘需要的超定方程组
Ax = b

*/
bool OdomCalib::Add_Data(Eigen::Vector3d Odom, Eigen::Vector3d scan)
{

    if (now_len < INT_MAX) {
        //TODO: 构建超定方程组
        Eigen::Matrix<double, 1, 3> odom_data;
        odom_data(0, 0) = Odom[0];
        odom_data(0, 1) = Odom[1];
        odom_data(0, 2) = Odom[2];

        for (size_t i = 0; i < 3; i++) {
            A.block<1, 3>(now_len * 3 + i, 3 * i) = odom_data;
        }
        b.block<3, 1>(now_len * 3, 0) = scan;
        //end of TODO
        now_len++;
        return true;
    } else {
        return false;
    }
}

/*
 * TODO:
 * 求解线性最小二乘Ax=b
 * 返回得到的矫正矩阵
*/
Eigen::Matrix3d OdomCalib::Solve()
{
    Eigen::Matrix3d correct_matrix;

    //TODO: 求解线性最小二乘
    Eigen::Matrix<double, 9, 1> params;
    // params = (A.transpose() * A).inverse() * A.transpose() * b;
    params = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    std::cout << params.transpose() << std::endl;
    for (size_t i = 0; i < 3; i++) {
        correct_matrix.block<1, 3>(i, 0) = params.block<3, 1>(i * 3, 0).transpose();
    }
    //end of TODO

    return correct_matrix;
}

/* 用于判断数据是否满
 * 数据满即可以进行最小二乘计算
*/
bool OdomCalib::is_full()
{
    if (now_len % data_len == 0 && now_len >= 1) {
        now_len = data_len;
        return true;
    } else
        return false;
}

/*
 * 数据清零
*/
void OdomCalib::set_data_zero()
{
    A.setZero();
    b.setZero();
}
