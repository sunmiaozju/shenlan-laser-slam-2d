#ifndef MYSLAM_G2O_TYPES_H
#define MYSLAM_G2O_TYPES_H

#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/slam3d/se3quat.h>

#include <gaussian_newton.h>

using namespace std;
using namespace g2o;

class my2dSlamVertex : public BaseVertex<3, Eigen::Vector3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    my2dSlamVertex() {}
    virtual bool read(std::istream& is) {}
    virtual bool write(std::ostream& os) const {}

    virtual void setToOriginImpl()
    {
        _estimate << 0, 0, 0;
    }
    virtual void oplusImpl(const double* update)
    {
        _estimate += Eigen::Vector3d(update);
    }
};

class my2dSlamEdge : public BaseBinaryEdge<3, Eigen::Vector3d, my2dSlamVertex, my2dSlamVertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void computeError()
    {
        const my2dSlamVertex* v1 = static_cast<const my2dSlamVertex*>(_vertices[0]);
        const my2dSlamVertex* v2 = static_cast<const my2dSlamVertex*>(_vertices[1]);

        const Eigen::Vector3d xi = v1->estimate();
        const Eigen::Vector3d xj = v2->estimate();

        Eigen::Matrix3d Xi = PoseToTrans(xi);
        Eigen::Matrix3d Xj = PoseToTrans(xj);
        Eigen::Matrix3d Z = PoseToTrans(_measurement);

        Eigen::Matrix3d Ei = Z.inverse() * Xi.inverse() * Xj;
        _error = TransToPose(Ei);
    }

    virtual void linearizeOplus()
    {
        const my2dSlamVertex* v1 = static_cast<const my2dSlamVertex*>(_vertices[0]);
        const my2dSlamVertex* v2 = static_cast<const my2dSlamVertex*>(_vertices[1]);

        Eigen::Vector3d xi = v1->estimate();
        Eigen::Vector3d xj = v2->estimate();
        Eigen::Vector3d z = _measurement;

        Eigen::Matrix3d trans_xi = PoseToTrans(xi);
        Eigen::Matrix3d trans_xj = PoseToTrans(xj);
        Eigen::Matrix3d trans_z = PoseToTrans(_measurement);

        Eigen::Matrix2d R_i = trans_xi.block(0, 0, 2, 2);
        Eigen::Matrix2d R_j = trans_xj.block(0, 0, 2, 2);
        Eigen::Matrix2d R_ij = trans_z.block(0, 0, 2, 2);

        Eigen::Vector2d t_i = xi.block(0, 0, 2, 1);
        Eigen::Vector2d t_j = xj.block(0, 0, 2, 1);
        Eigen::Vector2d t_ij = z.block(0, 0, 2, 1);

        _jacobianOplusXi.setZero();
        _jacobianOplusXi.block(0, 0, 2, 2) = -R_ij.transpose() * R_i.transpose();
        _jacobianOplusXi(2, 2) = -1;
        Eigen::Matrix2d derivative_Ri_theta;
        derivative_Ri_theta << -sin(xi[2]), cos(xi[2]), -cos(xi[2]), -sin(xi[2]);
        _jacobianOplusXi.block(0, 2, 2, 1) = R_ij.transpose() * derivative_Ri_theta * (t_j - t_i);

        _jacobianOplusXj.setZero();
        _jacobianOplusXj.block(0, 0, 2, 2) = R_ij.transpose() * R_i.transpose();
        _jacobianOplusXj(2, 2) = 1;
    }

    virtual bool read(istream& in) {}
    virtual bool write(ostream& out) const {}
};

#endif // MYSLAM_G2O_TYPES_H