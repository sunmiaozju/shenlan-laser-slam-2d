#include "imls_icp.h"

//pcl::visualization::CloudViewer g_cloudViewer("view-view");

IMLSICPMatcher::IMLSICPMatcher(void)
{
    m_r = 0.03;
    m_h = 0.10;
    m_Iterations = 10;

    m_PtID = 0;

    m_pTargetKDTree = m_pSourceKDTree = NULL;
}

//构造函数．
IMLSICPMatcher::IMLSICPMatcher(double _r, double _h, int _iter)
{
    m_r = _r;
    m_h = _h;
    m_Iterations = _iter;
}

//析构函数，释放内存nearNormal．
IMLSICPMatcher::~IMLSICPMatcher(void)
{
    if (m_pTargetKDTree != NULL)
        delete m_pTargetKDTree;

    if (m_pSourceKDTree != NULL)
        delete m_pSourceKDTree;
}

void IMLSICPMatcher::setIterations(int _iter)
{
    m_Iterations = _iter;
}

//去除非法数据
void IMLSICPMatcher::RemoveNANandINFData(std::vector<Eigen::Vector2d>& _input)
{
    //去除非法数据．
    for (std::vector<Eigen::Vector2d>::iterator it = _input.begin(); it != _input.end();) {
        Eigen::Vector2d tmpPt = *it;
        if (std::isnan(tmpPt(0)) || std::isnan(tmpPt(1)) || std::isinf(tmpPt(0)) || std::isinf(tmpPt(1))) {
            it = _input.erase(it);
        } else {
            it++;
        }
    }
}

void IMLSICPMatcher::setSourcePointCloud(pcl::PointCloud<pcl::PointXYZ>& pcl_cloud)
{
    std::vector<Eigen::Vector2d> source_ptCloud;
    for (int i = 0; i < pcl_cloud.size(); i++) {
        source_ptCloud.push_back(Eigen::Vector2d(pcl_cloud[i].x, pcl_cloud[i].y));
    }
    setSourcePointCloud(source_ptCloud);
}

void IMLSICPMatcher::setSourcePointCloud(std::vector<Eigen::Vector2d>& _source_pcloud)
{
    m_sourcePointCloud = _source_pcloud;

    //去除非法数据
    RemoveNANandINFData(m_sourcePointCloud);

    // std::cout <<"Source Pt Cloud:"<<m_sourcePointCloud.size()<<std::endl;
}

void IMLSICPMatcher::setSourcePointCloudNormals(std::vector<Eigen::Vector2d>& _normals)
{
    m_sourcePtCloudNormals = _normals;
}

//设置目标点云．
void IMLSICPMatcher::setTargetPointCloud(pcl::PointCloud<pcl::PointXYZ>& pcl_cloud)
{
    std::vector<Eigen::Vector2d> target_ptCloud;
    for (int i = 0; i < pcl_cloud.size(); i++) {
        target_ptCloud.push_back(Eigen::Vector2d(pcl_cloud[i].x, pcl_cloud[i].y));
    }
    setTargetPointCloud(target_ptCloud);
}

//本函数会自动重新生nearNormal成kd树．
void IMLSICPMatcher::setTargetPointCloud(std::vector<Eigen::Vector2d>& _target_pcloud)
{
    m_targetPointCloud = _target_pcloud;

    if (m_pTargetKDTree != NULL) {
        delete m_pTargetKDTree;
        m_pTargetKDTree = NULL;
    }

    RemoveNANandINFData(m_targetPointCloud);

    //构建kd树．
    if (m_pTargetKDTree == NULL) {
        m_targetKDTreeDataBase.resize(2, m_targetPointCloud.size());
        for (int i = 0; i < m_targetPointCloud.size(); i++) {
            m_targetKDTreeDataBase(0, i) = m_targetPointCloud[i](0);
            m_targetKDTreeDataBase(1, i) = m_targetPointCloud[i](1);
        }
        m_pTargetKDTree = Nabo::NNSearchD::createKDTreeLinearHeap(m_targetKDTreeDataBase);
    }

    //设置需要重新计算法向量
    m_isGetNormals = false;
}

//IMLS函数，主要用来进行曲面投影．
//可以认为是xi在曲面上的高度．
//用target_sourcePtcloud构造一个kd树．
bool IMLSICPMatcher::ImplicitMLSFunction(Eigen::Vector2d x,
    double& height)
{
    double weightSum = 0.0;
    double projSum = 0.0;

    //创建KD树
    if (m_pTargetKDTree == NULL) {
        m_targetKDTreeDataBase.resize(2, m_targetPointCloud.size());
        for (int i = 0; i < m_targetPointCloud.size(); i++) {
            m_targetKDTreeDataBase(0, i) = m_targetPointCloud[i](0);
            m_targetKDTreeDataBase(1, i) = m_targetPointCloud[i](1);
        }
        m_pTargetKDTree = Nabo::NNSearchD::createKDTreeLinearHeap(m_targetKDTreeDataBase);
    }

    // 找到位于点x附近(m_r)的所有的点云
    int searchNumber = 20;
    Eigen::VectorXi nearIndices(searchNumber);
    Eigen::VectorXd nearDist2(searchNumber);

    //找到某一个点的最近邻．
    //搜索searchNumber个最近邻
    //下标储存在nearIndices中，距离储存在nearDist2中．
    //最大搜索距离为m_r
    m_pTargetKDTree->knn(x, nearIndices, nearDist2, searchNumber, 0,
        Nabo::NNSearchD::SORT_RESULTS | Nabo::NNSearchD::ALLOW_SELF_MATCH | Nabo::NNSearchD::TOUCH_STATISTICS,
        m_h);

    std::vector<Eigen::Vector2d> nearPoints;
    std::vector<Eigen::Vector2d> nearNormals;
    for (int i = 0; i < searchNumber; i++) {
        //说明最近邻是合法的．
        if (nearDist2(i) < std::numeric_limits<double>::infinity() && std::isinf(nearDist2(i)) == false && std::isnan(nearDist2(i)) == false) {
            //该最近邻在原始数据中的下标．
            int index = nearIndices(i);

            Eigen::Vector2d tmpPt(m_targetKDTreeDataBase(0, index), m_targetKDTreeDataBase(1, index));

            //是否为inf
            if (std::isinf(tmpPt(0)) || std::isinf(tmpPt(1)) || std::isnan(tmpPt(0)) || std::isnan(tmpPt(1))) {
                continue;
            }

            Eigen::Vector2d normal;
            normal = m_targetPtCloudNormals[index];

            //如果对应的点没有法向量，则不要．
            if (std::isinf(normal(0)) || std::isinf(normal(1)) || std::isnan(normal(0)) || std::isnan(normal(1))) {
                continue;
            }

            nearPoints.push_back(tmpPt);
            nearNormals.push_back(normal);
        } else {
            break;
        }
    }

    //如果nearPoints小于３个，则认为没有匹配点．
    if (nearPoints.size() < 3) {
        return false;
    }

    //TODO
    //根据函数进行投影．计算height，即ppt中的I(x)
    Eigen::Vector2d v;
    double w, w_sum = 0, sum = 0;
    for (size_t i = 0; i < nearPoints.size(); i++) {
        v = x - nearPoints[i];
        w = exp(-((v[0] * v[0] + v[1] * v[1])) / (m_h * m_h));
        w_sum += w;
        sum += w * v.dot(nearNormals[i]);
    }
    height = sum / w_sum;
    //end of TODO

    return true;
}

/**
 * @brief IMLSICPMatcher::projSourcePtToSurface
 * 此函数的目的是把source_cloud中的点云，投影到对应的surface上．
   即为每一个当前帧点云in_cloud的激光点计算匹配点和对应的法向量
   即in_cloud和out_cloud进行匹配，同时得到out_normal
   注意：本函数应该删除in_cloud内部找不到匹配值的点．
 * @param in_cloud          当前帧点云
 * @param out_cloud         当前帧的匹配点云
 * @param out_normal        匹配点云对应的法向量．
 */
void IMLSICPMatcher::projSourcePtToSurface(
    std::vector<Eigen::Vector2d>& in_cloud,
    std::vector<Eigen::Vector2d>& out_cloud,
    std::vector<Eigen::Vector2d>& out_normal)
{
    out_cloud.clear();
    out_normal.clear();

    for (std::vector<Eigen::Vector2d>::iterator it = in_cloud.begin(); it != in_cloud.end();) {
        Eigen::Vector2d xi = *it;

        //找到在target_cloud中的最近邻
        //包括该点和下标．
        int K = 1;
        Eigen::VectorXi indices(K);
        Eigen::VectorXd dist2(K);
        m_pTargetKDTree->knn(xi, indices, dist2);

        Eigen::Vector2d nearXi = m_targetKDTreeDataBase.col(indices(0));
        int nor_count = 0;
        //为最近邻计算法向量．－－进行投影的时候，使用统一的法向量．
        Eigen::Vector2d nearNormal = m_targetPtCloudNormals[indices(0)];
        for (size_t i = 0; i < m_targetPtCloudNormals.size(); i++) {
            Eigen::Vector2d nor = m_targetPtCloudNormals[i];
            if (std::isinf(nor(0)) || std::isinf(nor(1)) || std::isnan(nor(0)) || std::isnan(nor(1))) {
                nor_count++;
            }
        }
        // cout << "size(): " << m_targetPtCloudNormals.size() << " nor_count: " << nor_count << endl;

        //如果对应的点没有法向量，也认为没有匹配点．因此直接不考虑．
        if (std::isinf(nearNormal(0)) || std::isinf(nearNormal(1)) || std::isnan(nearNormal(0)) || std::isnan(nearNormal(1))) {
            it = in_cloud.erase(it);
            // cout << "erase: wrong normal" << nearNormal << endl;
            continue;
        }

        //如果距离太远，则说明没有匹配点．因此可以不需要进行投影，直接去除．
        if (dist2(0) > m_h * m_h) {
            it = in_cloud.erase(it);
            // cout << "erase: distance too far" << endl;
            continue;
        }

        //进行匹配
        double height;
        // std::cout << "---pro "
        //           << " in "
        //           << " pro ---" << std::endl;
        if (ImplicitMLSFunction(xi, height) == false) {
            it = in_cloud.erase(it);
            // cout << "erase: cal height false" << endl;
            continue;
        }
        // cout << "height: " << height << endl;
        // std::cout << "---pro "
        //           << " out "
        //           << " pro ---" << std::endl;

        if (std::isnan(height)) {
            std::cout << "proj:this is not possible" << std::endl;
            it = in_cloud.erase(it);
            continue;
        }

        if (std::isinf(height)) {
            std::cout << "proj:this is inf,not possible" << std::endl;
            it = in_cloud.erase(it);
            continue;
        }

        Eigen::Vector2d yi;
        //TODO
        //计算yi．
        yi = xi - height * nearNormal;
        //end of TODO

        out_cloud.push_back(yi);
        out_normal.push_back(nearNormal);

        it++;
    }
    // cout << in_cloud.size() << "   " << out_cloud.size() << "   " << out_normal.size() << endl;
}

//已知对应点对和法向量的时候，求解相对位姿．
//source_cloud在ref_cloud下的位姿．
//具体的算法，可以参考point-to-line ICP论文．
bool IMLSICPMatcher::SolveMotionEstimationProblem(std::vector<Eigen::Vector2d>& source_cloud,
    std::vector<Eigen::Vector2d>& ref_cloud,
    std::vector<Eigen::Vector2d>& ref_normals,
    Eigen::Matrix3d& deltaTrans)
{
    Eigen::Matrix4d M;
    M.setZero();
    Eigen::Matrix<double, 1, 4> gt; //gt是个行向量．
    gt.setZero();

    for (int i = 0; i < source_cloud.size(); i++) {
        //点p-source point
        Eigen::Vector2d p = source_cloud[i];

        //target-point
        Eigen::Vector2d refPt = ref_cloud[i];

        //ref对应的normal
        Eigen::Vector2d ni = ref_normals[i];

        //加权矩阵
        //对于p-p来说，Ci =wi * I
        //对于point-line来说，Ci =wi *  n*n^T
        Eigen::Matrix2d Ci = ni * ni.transpose();

        //构造M矩阵
        Eigen::Matrix<double, 2, 4> Mi;
        Mi << 1, 0, p(0), -p(1),
            0, 1, p(1), p(0);
        M += Mi.transpose() * Ci * Mi;

        Eigen::Matrix<double, 1, 4> gti;
        gti = -2 * refPt.transpose() * Ci * Mi;

        gt += gti;
    }

    //g是个列向量
    Eigen::Matrix<double, 4, 1> g = gt.transpose();

    //构建完了M矩阵和g向量．
    //在后续的求解过程中，基本上都使用的是2*M,因此直接令M = 2*M
    M = 2 * M;

    //M(实际是2*M,下面等同)矩阵能分为４个部分
    Eigen::Matrix2d A, B, D;
    A = M.block(0, 0, 2, 2);
    B = M.block(0, 2, 2, 2);
    D = M.block(2, 2, 2, 2);

    //论文中还引入了S和SA矩阵．
    Eigen::Matrix2d S, SA;
    S = D - B.transpose() * A.inverse() * B;
    SA = S.determinant() * S.inverse();

    //目前所有的式子已经可以构建多项式系数了．
    //式31右边p(\lambda)的系数
    //p(\lambda)的系数为：
    Eigen::Vector3d p_coffcient;
    p_coffcient << S.determinant(), 2 * (S(0, 0) + S(1, 1)), 4;

    //论文中式(31)左边的系数(a x^2 + b x + c)为：
    double a, b, c;
    Eigen::Matrix4d tmpMatrix;
    tmpMatrix.block(0, 0, 2, 2) = A.inverse() * B * SA * SA.transpose() * B.transpose() * A.inverse().transpose();
    tmpMatrix.block(0, 2, 2, 2) = -A.inverse() * B * SA * SA.transpose();
    tmpMatrix.block(2, 0, 2, 2) = tmpMatrix.block(0, 2, 2, 2).transpose();
    tmpMatrix.block(2, 2, 2, 2) = SA * SA.transpose();

    c = g.transpose() * tmpMatrix * g;

    tmpMatrix.block(0, 0, 2, 2) = A.inverse() * B * SA * B.transpose() * A.inverse().transpose();
    tmpMatrix.block(0, 2, 2, 2) = -A.inverse() * B * SA;
    tmpMatrix.block(2, 0, 2, 2) = tmpMatrix.block(0, 2, 2, 2).transpose();
    tmpMatrix.block(2, 2, 2, 2) = SA;
    b = 4 * g.transpose() * tmpMatrix * g;

    tmpMatrix.block(0, 0, 2, 2) = A.inverse() * B * B.transpose() * A.inverse().transpose();
    tmpMatrix.block(0, 2, 2, 2) = -A.inverse() * B;
    tmpMatrix.block(2, 0, 2, 2) = tmpMatrix.block(0, 2, 2, 2).transpose();
    tmpMatrix.block(2, 2, 2, 2) = Eigen::Matrix2d::Identity();
    a = 4 * g.transpose() * tmpMatrix * g;

    //把式31的等式两边进行合并，得到一个4次多项式．５个系数．
    Eigen::VectorXd poly_coffi(5);
    poly_coffi(0) = c - p_coffcient(0) * p_coffcient(0);
    poly_coffi(1) = b - 2 * p_coffcient(0) * p_coffcient(1);
    poly_coffi(2) = a - (p_coffcient(1) * p_coffcient(1) + 2 * p_coffcient(0) * p_coffcient(2));
    poly_coffi(3) = -2 * p_coffcient(1) * p_coffcient(2);
    poly_coffi(4) = -p_coffcient(2) * p_coffcient(2);

    for (int i = 0; i < 5; i++) {
        if (std::isnan(poly_coffi(i))) {
            std::cout << "Error, This should not happen" << std::endl;
        }
    }

    //进行多项式的求解，得到对应的lambda．
    double lambda;
    if (SolverFourthOrderPolynomial(poly_coffi, lambda) == false) {
        std::cout << "Polynomial Solve Failed" << std::endl;
        return false;
    }

    //得到lambda之后，根据式24．
    Eigen::Matrix4d W;
    W.setZero();
    W.block(2, 2, 2, 2) = Eigen::Matrix2d::Identity();

    //Eigen::Vector4d res = -(M + 2 *lambda * W).inverse().transpose() * g;
    Eigen::Vector4d res = -(M + 2 * lambda * W).inverse() * g;

    //转换成旋转矩阵
    double theta = std::atan2(res(3), res(2));
    deltaTrans << cos(theta), -sin(theta), res(0),
        sin(theta), cos(theta), res(1),
        0, 0, 1;
    return true;
}

/**
 * @brief IMLSICPMatcher::ComputeNormal
 * 计算法向量
 * @param nearPoints    某个点周围的所有激光点
 * @return
 */
Eigen::Vector2d IMLSICPMatcher::ComputeNormal(std::vector<Eigen::Vector2d>& nearPoints)
{
    Eigen::Vector2d normal;

    //TODO
    //根据周围的激光点计算法向量，参考ppt中NICP计算法向量的方法
    Eigen::Vector2d center(0, 0);
    for (size_t i = 0; i < nearPoints.size(); i++) {
        center = center + nearPoints[i];
    }
    center = center / nearPoints.size();
    Eigen::Matrix2d m;
    for (size_t i = 0; i < nearPoints.size(); i++) {
        m = m + (nearPoints[i] - center) * (nearPoints[i] - center).transpose();
    }
    m = m / nearPoints.size();
    // A = V * D * VT.
    Eigen::EigenSolver<Eigen::Matrix2d> es(m);
    Eigen::Matrix2d D = es.pseudoEigenvalueMatrix();
    Eigen::Matrix2d V = es.pseudoEigenvectors();
    if (D(0, 0) > D(1, 1)) {
        normal = V.col(1);
    } else {
        normal = V.col(0);
    }
    cout << "The  matrix M is:" << endl
         << m << endl;
    cout << "The pseudo-eigenvalue matrix D is:" << endl
         << D << endl;
    cout << "The pseudo-eigenvector matrix V is:" << endl
         << V << endl;
    cout << "Finally, V * D * V^(-1) = " << endl
         << V * D * V.inverse() << endl;
    //end of TODO
    return normal;
}
// cout << normal << endl;
// cout << "The  matrix M is:" << endl
//      << m << endl;
// cout << "The pseudo-eigenvalue matrix D is:" << endl
//      << D << endl;
// cout << "The pseudo-eigenvector matrix V is:" << endl
//      << V << endl;
// cout << "Finally, V * D * V^(-1) = " << endl
//      << V * D * V.inverse() << endl;
/**
 * @brief IMLSICPMatcher::Match
 * 最终使用的ICP匹配函数．
 * @param finalResult
 * @param covariance
 * @return
 */
bool IMLSICPMatcher::Match(Eigen::Matrix3d& finalResult,
    Eigen::Matrix3d& covariance)
{
    //如果没有设置法向量，则自动进行计算．

    if (m_isGetNormals == false) {
        //自动计算target pointcloud中每个点的法向量
        m_targetPtCloudNormals.clear();
        for (int i = 0; i < m_targetPointCloud.size(); i++) {
            Eigen::Vector2d xi = m_targetPointCloud[i];

            int K = 20;
            Eigen::VectorXi indices(K);
            Eigen::VectorXd dist2(K);
            int num = m_pTargetKDTree->knn(xi, indices, dist2, K, 0.0,
                Nabo::NNSearchD::SORT_RESULTS | Nabo::NNSearchD::ALLOW_SELF_MATCH | Nabo::NNSearchD::TOUCH_STATISTICS,
                0.15);

            std::vector<Eigen::Vector2d> nearPoints;
            for (int ix = 0; ix < K; ix++) {
                // cout << ix << "----first dist:  " << dist2(ix) << endl;
                if (dist2(ix) < std::numeric_limits<double>::infinity() && std::isinf(dist2(ix)) == false) {
                    nearPoints.push_back(m_targetKDTreeDataBase.col(indices(ix)));
                } else
                    break;
            }

            //计算法向量
            Eigen::Vector2d normal;
            // cout << "-----now nearPoints.size() " << nearPoints.size() << endl;
            if (nearPoints.size() > 3) {
                //计算法向量
                normal = ComputeNormal(nearPoints);
            } else {
                normal(0) = normal(1) = std::numeric_limits<double>::infinity();
            }
            m_targetPtCloudNormals.push_back(normal);
        }
    }

    //初始化估计值．
    Eigen::Matrix3d result;
    result.setIdentity();
    covariance.setIdentity();
    // std::cout << "---0---" << std::endl;

    for (int i = 0; i < m_Iterations; i++) {
        //根据当前估计的位姿对原始点云进行转换．
        std::vector<Eigen::Vector2d> in_cloud;
        for (int ix = 0; ix < m_sourcePointCloud.size(); ix++) {
            Eigen::Vector3d origin_pt;
            origin_pt << m_sourcePointCloud[ix], 1;

            Eigen::Vector3d now_pt = result * origin_pt;
            in_cloud.push_back(Eigen::Vector2d(now_pt(0), now_pt(1)));
        }

        //把sourceCloud中的点投影到targetCloud组成的平面上
        //对应的投影点即为sourceCloud的匹配点．
        //每次转换完毕之后，都需要重新计算匹配点．
        //这个函数会得到对应的匹配点．
        //本次匹配会自动删除in_cloud内部的一些找不到匹配点的点．
        //因此，这个函数出来之后，in_cloud和ref_cloud是一一匹配的．
        std::vector<Eigen::Vector2d> ref_cloud;
        std::vector<Eigen::Vector2d> ref_normal;
        // std::cout << "in_cloud.size(): " << in_cloud.size() << " for ---" << std::endl;

        projSourcePtToSurface(in_cloud,
            ref_cloud,
            ref_normal);
        // std::cout << "---for " << i << " for ---" << std::endl;

        if (in_cloud.size() < 5 || ref_cloud.size() < 5) {
            std::cout << "Not Enough Correspondence: i " << i << ", " << in_cloud.size() << "," << ref_cloud.size() << std::endl;
            std::cout << "ICP Iterations Failed!!" << std::endl;
            return false;
        }

        //计算帧间位移．从当前的source -> target
        Eigen::Matrix3d deltaTrans;
        bool flag = SolveMotionEstimationProblem(in_cloud,
            ref_cloud,
            ref_normal, deltaTrans);

        if (flag == false) {
            std::cout << "ICP Iterations Failed!!!!" << std::endl;
            return false;
        }

        //更新位姿．
        result = deltaTrans * result;

        //迭代条件是否满足．
        double deltadist = std::sqrt(std::pow(deltaTrans(0, 2), 2) + std::pow(deltaTrans(1, 2), 2));
        double deltaAngle = std::atan2(deltaTrans(1, 0), deltaTrans(0, 0));

        //如果迭代条件允许，则直接退出．
        if (deltadist < 0.001 && deltaAngle < (0.01 / 57.295)) {
            break;
        }
    }

    finalResult = result;
    return true;
}

//调用Eigen求解四次多项式的第一个非零实根
bool IMLSICPMatcher::SolverFourthOrderPolynomial(Eigen::VectorXd& p_coffi,
    double& lambda)
{
    Eigen::PolynomialSolver<double, 4> polySolve(p_coffi);

    Eigen::Matrix<std::complex<double>, 4, 1, 0, 4, 1> roots = polySolve.roots();

    bool isAssigned = false;
    double finalRoot = 0.0;

    //找到第一个非零实根--有可能不止一个，因为有优化空间．
    for (int i = 0; i < roots.size(); i++) {
        //如果是虚根，则不要．
        if (roots(i).imag() != 0)
            continue;

        //如果是非零实根，则选择．
        if (isAssigned == false || roots(i).real() > finalRoot) {
            isAssigned = true;
            finalRoot = roots(i).real();
        }
    }

    lambda = finalRoot;
    return isAssigned;
}
