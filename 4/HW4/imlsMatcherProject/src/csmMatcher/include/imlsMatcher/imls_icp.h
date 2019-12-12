#ifndef IMLS_ICP_H
#define IMLS_ICP_H

#include <iostream>
#include <nabo/nabo.h>
#include <set>
#include <vector>

//#include <opencv2/opencv.hpp>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/Geometry>
#include <eigen3/unsupported/Eigen/Polynomials>

#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/visualization/cloud_viewer.h>

#include <sys/time.h>

class IMLSICPMatcher {
public:
    IMLSICPMatcher();
    IMLSICPMatcher(double _r, double _h, int _iter);
    ~IMLSICPMatcher();

    void setIterations(int _iter);

    void setSourcePointCloud(std::vector<Eigen::Vector2d>& _source_pcloud);

    void setSourcePointCloud(pcl::PointCloud<pcl::PointXYZ>& pcl_cloud);

    void setSourcePointCloudNormals(std::vector<Eigen::Vector2d>& _normals);

    void setTargetPointCloudNormals(std::vector<Eigen::Vector2d>& _normals);

    void setTargetPointCloud(std::vector<Eigen::Vector2d>& _target_pcloud);

    void setTargetPointCloud(pcl::PointCloud<pcl::PointXYZ>& pcl_cloud);

    void createKDTreeUsingLocalMap(void);

    //IMLS函数，主要功能为进行把xi投影到表面上．
    bool ImplicitMLSFunction(Eigen::Vector2d x,
        double& height);

    //把点云进行投影，投影到表面surface之中．
    void projSourcePtToSurface(std::vector<Eigen::Vector2d>& in_cloud,
        std::vector<Eigen::Vector2d>& out_cloud,
        std::vector<Eigen::Vector2d>& out_normal);

    //求解四阶多项式
    bool SolverFourthOrderPolynomial(Eigen::VectorXd& p_coffi,
        double& lambda);

    //在已知匹配点的情况下，进行运动估计．
    bool SolveMotionEstimationProblem(std::vector<Eigen::Vector2d>& source_cloud,
        std::vector<Eigen::Vector2d>& ref_cloud,
        std::vector<Eigen::Vector2d>& ref_normals,
        Eigen::Matrix3d& deltaTrans);

    Eigen::Vector2d ComputeNormal(std::vector<Eigen::Vector2d>& nearPoints);

    bool Match(Eigen::Matrix3d& finalPose,
        Eigen::Matrix3d& covariance);

private:
    void RemoveNANandINFData(std::vector<Eigen::Vector2d>& _input);

    //目标点云和当前点云，目标点云为参考点云．
    std::vector<Eigen::Vector2d> m_sourcePointCloud, m_targetPointCloud;

    //目标点云的法向量
    std::vector<Eigen::Vector2d> m_sourcePtCloudNormals, m_targetPtCloudNormals;

    //所有的激光帧数据，每一个激光帧数据对应的点云的下标．
    std::map<int, std::vector<int>> m_LaserFrames;

    //指针
    Nabo::NNSearchD* m_pTargetKDTree;
    Nabo::NNSearchD* m_pSourceKDTree;

    Eigen::MatrixXd m_sourceKDTreeDataBase;
    Eigen::MatrixXd m_targetKDTreeDataBase;

    //迭代次数．
    int m_Iterations;

    //点云的id．
    int m_PtID;

    //LocalMap和Tree之间的下标偏移，Tree永远是从0开始．
    int m_offsetBetweenLocalMapAndTree;

    //含义见论文．用来计算权重．
    //m_r ~ 3*m_h
    double m_h, m_r;

    //用来判断是否需要自己进行计算．
    bool m_isGetNormals;
};

#endif // IMLS_ICP_H
