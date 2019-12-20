#ifndef GAUSSIAN_NEWTON_H
#define GAUSSIAN_NEWTON_H

#include <eigen3/Eigen/Core>
#include <vector>

typedef struct edge {
    int xi, xj;
    Eigen::Vector3d measurement;
    Eigen::Matrix3d infoMatrix;
} Edge;

Eigen::VectorXd LinearizeAndSolve(std::vector<Eigen::Vector3d>& Vertexs,
    std::vector<Edge>& Edges);

double ComputeError(std::vector<Eigen::Vector3d>& Vertexs,
    std::vector<Edge>& Edges);

inline void NormalAngle(double& angle)
{
    if (angle > M_PI)
        angle -= 2 * M_PI;
    else if (angle < -M_PI)
        angle += 2 * M_PI;
}

Eigen::Matrix3d PoseToTrans(Eigen::Vector3d x);
Eigen::Vector3d TransToPose(Eigen::Matrix3d trans);

#endif
