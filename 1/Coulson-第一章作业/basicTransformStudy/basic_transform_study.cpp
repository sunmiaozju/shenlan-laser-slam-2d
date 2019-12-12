#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
    // 机器人B在坐标系O中的坐标：
    Eigen::Vector3d B(3, 4, M_PI);

    // 坐标系B到坐标O的转换矩阵：
    Eigen::Matrix3d TOB;
    TOB << cos(B(2)), -sin(B(2)), B(0),
        sin(B(2)), cos(B(2)), B(1),
        0, 0, 1;

    // 坐标系O到坐标B的转换矩阵:
    Eigen::Matrix3d TBO = TOB.inverse();

    // 机器人A在坐标系O中的坐标：
    Eigen::Vector3d A(1, 3, -M_PI / 2);

    // 求机器人A在机器人B中的坐标：
    Eigen::Vector3d BA;
    // TODO 参照PPT
    // start your code here (5~10 lines)
    Eigen::Matrix3d TOA;
    TOA << cos(A(2)), -sin(A(2)), A(0),
        sin(A(2)), cos(A(2)), A(1),
        0, 0, 1;
    Eigen::Matrix3d TAB = TBO * TOA;
    BA[0] = TAB(0, 2);
    BA[1] = TAB(1, 2);
    BA[2] = atan2(TAB(1, 0), TAB(0, 0));
    // end your code here

    cout
        << "The right answer is BA: 2 1 1.5708" << endl;
    cout << "Your answer is BA: " << BA.transpose() << endl;

    return 0;
}
