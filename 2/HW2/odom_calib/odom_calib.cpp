#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <csm/csm_all.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <string>

using namespace std;

string scan_match_file = "./scan_match.txt";
string odom_file = "./odom.txt";

int main(int argc, char** argv)
{
    // 放置激光雷达的时间和匹配值 t_s s_x s_y s_th
    vector<vector<double>> s_data;
    // 放置轮速计的时间和左右轮角速度 t_r w_L w_R
    vector<vector<double>> r_data;

    ifstream fin_s(scan_match_file);
    ifstream fin_r(odom_file);
    if (!fin_s || !fin_r) {
        cerr << "请在有scan_match.txt和odom.txt的目录下运行此程序" << endl;
        return 1;
    }

    // 读取激光雷达的匹配值
    while (!fin_s.eof()) {
        double s_t, s_x, s_y, s_th;
        fin_s >> s_t >> s_x >> s_y >> s_th;
        s_data.push_back(vector<double>({ s_t, s_x, s_y, s_th }));
    }
    fin_s.close();

    // 读取两个轮子的角速度
    while (!fin_r.eof()) {
        double t_r, w_L, w_R;
        fin_r >> t_r >> w_L >> w_R;
        r_data.push_back(vector<double>({ t_r, w_L, w_R }));
    }
    fin_r.close();

    // 第一步：计算中间变量J_21和J_22
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    // 设置数据长度
    A.conservativeResize(5000, 2);
    b.conservativeResize(5000);
    A.setZero();
    b.setZero();

    size_t id_r = 0;
    size_t id_s = 0;
    double last_rt = r_data[0][0];
    double w_Lt = 0;
    double w_Rt = 0;
    while (id_s < 5000) {
        // 激光的匹配信息
        const double& s_t = s_data[id_s][0];
        const double& s_th = s_data[id_s][3];
        // 里程计信息
        const double& r_t = r_data[id_r][0];
        const double& w_L = r_data[id_r][1];
        const double& w_R = r_data[id_r][2];
        ++id_r;
        // 在2帧激光匹配时间内进行里程计角度积分
        if (r_t < s_t) {
            double dt = r_t - last_rt;
            w_Lt += w_L * dt;
            w_Rt += w_R * dt;
            last_rt = r_t;
        } else {
            double dt = s_t - last_rt;
            w_Lt += w_L * dt;
            w_Rt += w_R * dt;
            last_rt = s_t;
            // 填充A, b矩阵
            //TODO: (3~5 lines)
            A(id_s, 0) = w_Lt;
            A(id_s, 1) = w_Rt;
            b[id_s] = s_th;
            //end of TODO
            w_Lt = 0;
            w_Rt = 0;
            ++id_s;
        }
    }
    // 进行最小二乘求解
    Eigen::Vector2d J21J22;
    //TODO: (1~2 lines)
    J21J22 = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    //end of TODO
    const double& J21 = J21J22(0);
    const double& J22 = J21J22(1);
    cout << "J21: " << J21 << endl;
    cout << "J22: " << J22 << endl;

    // 第二步，求解轮间距b
    Eigen::VectorXd C;
    Eigen::VectorXd S;
    // 设置数据长度
    C.conservativeResize(10000);
    S.conservativeResize(10000);
    C.setZero();
    S.setZero();

    id_r = 0;
    id_s = 0;
    last_rt = r_data[0][0];
    double th = 0;
    double cx = 0;
    double cy = 0;
    while (id_s < 5000) {
        // 激光的匹配信息
        const double& s_t = s_data[id_s][0];
        const double& s_x = s_data[id_s][1];
        const double& s_y = s_data[id_s][2];
        // 里程计信息
        const double& r_t = r_data[id_r][0];
        const double& w_L = r_data[id_r][1];
        const double& w_R = r_data[id_r][2];
        ++id_r;
        // 在2帧激光匹配时间内进行里程计位置积分
        if (r_t < s_t) {
            double dt = r_t - last_rt;
            cx += 0.5 * (-J21 * w_L * dt + J22 * w_R * dt) * cos(th);
            cy += 0.5 * (-J21 * w_L * dt + J22 * w_R * dt) * sin(th);
            th += (J21 * w_L + J22 * w_R) * dt;
            last_rt = r_t;
        } else {
            double dt = s_t - last_rt;
            cx += 0.5 * (-J21 * w_L * dt + J22 * w_R * dt) * cos(th);
            cy += 0.5 * (-J21 * w_L * dt + J22 * w_R * dt) * sin(th);
            th += (J21 * w_L + J22 * w_R) * dt;
            last_rt = s_t;
            // 填充C, S矩阵
            //TODO: (4~5 lines)
            C[id_s * 2] = cx;
            C[id_s * 2 + 1] = cy;
            S[id_s * 2] = s_x;
            S[id_s * 2 + 1] = s_y;
            //end of TODO
            cx = 0;
            cy = 0;
            th = 0;
            ++id_s;
        }
    }
    // 进行最小二乘求解，计算b, r_L, r_R
    double b_wheel;
    double r_L;
    double r_R;
    //TODO: (3~5 lines)
    b_wheel = C.colPivHouseholderQr().solve(S)[0];
    r_L = -J21 * b_wheel;
    r_R = J22 * b_wheel;
    //end of TODO
    cout << "b: " << b_wheel << endl;
    cout << "r_L: " << r_L << endl;
    cout << "r_R: " << r_R << endl;

    cout << "参考答案：轮间距b为0.6m左右，两轮半径为0.1m左右" << endl;

    return 0;
}
