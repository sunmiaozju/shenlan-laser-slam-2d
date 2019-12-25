#include "occupany_mapping.h"
#include "geometry_msgs/Point32.h"
#include "nav_msgs/GetMap.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include <climits>
#include <cmath>
#include <queue>

using namespace std;

/**
 * Increments all the grid cells from (x0, y0) to (x1, y1);
 * //不包含(x1,y1)
 * 2D画线算法　来进行计算两个点之间的grid cell
 * @param x0
 * @param y0
 * @param x1
 * @param y1
 */
std::vector<GridIndex> TraceLine(int x0, int y0, int x1, int y1)
{
    GridIndex tmpIndex;
    std::vector<GridIndex> gridIndexVector;

    bool steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
        std::swap(x0, y0);
        std::swap(x1, y1);
    }
    if (x0 > x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    int deltaX = x1 - x0;
    int deltaY = abs(y1 - y0);
    int error = 0;
    int ystep;
    int y = y0;

    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }

    int pointX;
    int pointY;
    for (int x = x0; x <= x1; x++) {
        if (steep) {
            pointX = y;
            pointY = x;
        } else {
            pointX = x;
            pointY = y;
        }

        error += deltaY;

        if (2 * error >= deltaX) {
            y += ystep;
            error -= deltaX;
        }

        //不包含最后一个点．
        if (pointX == x1 && pointY == y1)
            continue;

        //保存所有的点
        tmpIndex.SetIndex(pointX, pointY);

        gridIndexVector.push_back(tmpIndex);
    }

    return gridIndexVector;
}

void SetMapParams(void)
{
    mapParams.width = 1000;
    mapParams.height = 1000;
    mapParams.resolution = 0.05;

    mapParams.origin_x = 0.0;
    mapParams.origin_y = 0.0;

    pMap = new int[mapParams.width * mapParams.height];

    //地图的原点，在地图的正中间
    mapParams.offset_x = 500;
    mapParams.offset_y = 500;

    // 覆盖栅格建图算法需要的参数
    //每次被击中的log变化值，
    mapParams.log_free = -1;
    mapParams.log_occ = 2;

    //每个栅格的最大最小值．
    mapParams.log_max = 100.0;
    mapParams.log_min = 0.0;

    //计数建图算法需要的参数
    //每个栅格被激光击中的次数
    pMapHits = new unsigned long[mapParams.width * mapParams.height];
    //每个栅格被激光通过的次数
    pMapMisses = new unsigned long[mapParams.width * mapParams.height];

    //TSDF建图算法需要的参数
    pMapW = new unsigned long[mapParams.width * mapParams.height];
    pMapTSDF = new double[mapParams.width * mapParams.height];

    //初始化
    for (int i = 0; i < mapParams.width * mapParams.height; i++) {
        pMap[i] = 50;
        pMapHits[i] = 0;
        pMapMisses[i] = 0;
        pMapW[i] = 0;
        pMapTSDF[i] = -1;
    }
}

//从世界坐标系转换到栅格坐标系
GridIndex ConvertWorld2GridIndex(double x, double y)
{
    GridIndex index;

    index.x = std::ceil((x - mapParams.origin_x) / mapParams.resolution) + mapParams.offset_x;
    index.y = std::ceil((y - mapParams.origin_y) / mapParams.resolution) + mapParams.offset_y;

    return index;
}

int GridIndexToLinearIndex(GridIndex index)
{
    int linear_index;
    linear_index = index.y + index.x * mapParams.width;
}

//判断index是否有效
bool isValidGridIndex(GridIndex index)
{
    if (index.x >= 0 && index.x < mapParams.width && index.y >= 0 && index.y < mapParams.height)
        return true;

    return false;
}

void DestoryMap()
{
    if (pMap != NULL)
        delete pMap;
}

//
void OccupanyMapping(std::vector<GeneralLaserScan>& scans, std::vector<Eigen::Vector3d>& robot_poses)
{
    std::cout << "开始建图，请稍后..." << std::endl;
    //枚举所有的激光雷达数据

    for (int i = 0; i < scans.size(); i++) {
        GeneralLaserScan scan = scans[i];
        Eigen::Vector3d robotPose = robot_poses[i];

        //机器人的下标
        GridIndex robotIndex = ConvertWorld2GridIndex(robotPose(0), robotPose(1));

        for (int id = 0; id < scan.range_readings.size(); id++) {
            double dist = scan.range_readings[id];
            double angle = -scan.angle_readings[id]; // 激光雷达逆时针转，角度取反

            if (std::isinf(dist) || std::isnan(dist))
                continue;

            //计算得到该激光点的世界坐标系的坐标
            double theta = -robotPose(2); // 激光雷达逆时针转，角度取反
            double laser_x = dist * cos(angle);
            double laser_y = dist * sin(angle);

            double world_x = cos(theta) * laser_x - sin(theta) * laser_y + robotPose(0);
            double world_y = sin(theta) * laser_x + cos(theta) * laser_y + robotPose(1);

            //start of TODO 对对应的map的cell信息进行更新．（1,2,3题内容）

            GridIndex grid_x_y = ConvertWorld2GridIndex(world_x, world_y);

            if (isValidGridIndex(grid_x_y) == false)
                continue;

            GridIndex robotPose_grid = ConvertWorld2GridIndex(robotPose[0], robotPose[1]);

            //得到所有的被激光通过的index，并且更新栅格
            std::vector<GridIndex> miss_grids = TraceLine(robotPose_grid.x, robotPose_grid.y, grid_x_y.x, grid_x_y.y);

#ifdef PROBLEM_1
            // 更新被经过的点
            for (size_t j = 0; j < miss_grids.size(); j++) {
                GridIndex tmpIndex = miss_grids[j];
                int linear_index = GridIndexToLinearIndex(tmpIndex);
                pMap[linear_index] += mapParams.log_free;
                pMap[linear_index] = max(mapParams.log_min, double(pMap[linear_index]));
            }

            //更新被击中的点
            int linear_index = GridIndexToLinearIndex(grid_x_y);
            pMap[linear_index] += mapParams.log_occ;
            pMap[linear_index] = min(mapParams.log_max, double(pMap[linear_index]));
#endif // PROBLEM_1

#ifdef PROBLEM_2
            // 更新被经过的点
            for (size_t j = 0; j < miss_grids.size(); j++) {
                GridIndex tmpIndex = miss_grids[j];
                int linear_index = GridIndexToLinearIndex(tmpIndex);
                pMapMisses[linear_index]++;
            }

            //更新被击中的点
            int linear_index = GridIndexToLinearIndex(grid_x_y);
            pMapHits[linear_index]++;
#endif //PROBLEM_2

#ifdef PROBLEM_3
            // tsdf 截断距离
            double cut_off_dis = 2 * mapParams.resolution;
            double far_dis;

            // 计算远点
            far_dis = dist + 3 * cut_off_dis;

            double far_laser_x = far_dis * cos(angle);
            double far_laser_y = far_dis * sin(angle);

            double far_world_x = cos(theta) * far_laser_x - sin(theta) * far_laser_y + robotPose(0);
            double far_world_y = sin(theta) * far_laser_x + cos(theta) * far_laser_y + robotPose(1);

            GridIndex far_grid_x_y = ConvertWorld2GridIndex(far_world_x, far_world_y);

            std::vector<GridIndex> near_grids;
            // 如果增加的距离是远点超出地图范围，那么就是用激光点作为远点
            if (isValidGridIndex(far_grid_x_y) == false) {
                near_grids = TraceLine(robotPose_grid.x, robotPose_grid.y, grid_x_y.x, grid_x_y.y);
            } else {
                near_grids = TraceLine(robotPose_grid.x, robotPose_grid.y, far_grid_x_y.x, far_grid_x_y.y);
            }

            // 更新laser_dist附近的栅格
            for (size_t j = 0; j < near_grids.size(); j++) {
                GridIndex tmpIndex = near_grids[j];
                double grid_dis = sqrt(pow(tmpIndex.x - robotPose_grid.x, 2) + pow(tmpIndex.y - robotPose_grid.y, 2));

                // 从gridmap尺度转化为实际地图尺度
                grid_dis *= mapParams.resolution;

                // 计算tsdf
                double tsdf = max(-1.0, min(1.0, (dist - grid_dis) / cut_off_dis));

                int linearIndex = GridIndexToLinearIndex(tmpIndex);

                // 更新TSDF
                pMapTSDF[linearIndex] = (pMapW[linearIndex] * pMapTSDF[linearIndex] + tsdf) / (pMapW[linearIndex] + 1);
                pMapW[linearIndex] += 1;
                // cout << "sdf: " << dist - grid_dis << endl;
                // cout << "tsdf:     " << tsdf << endl;
                // cout << "grid_dis: " << grid_dis << endl;
            }
            // cout << "laser_dis: " << dist << endl;
            // cout << "----------" << endl;
#endif //PROBLEM_3

            //end of TODO
        }
    }

    //start of TODO 通过计数建图算法或TSDF算法对栅格进行更新（2,3题内容）
#ifdef PROBLEM_2
    for (int i = 0; i < mapParams.width * mapParams.height; i++) {
        if (pMapHits[i] + pMapMisses[i] == 0) {
            pMap[i] = 50;
            continue;
        }

        double ratio = pMapHits[i] * 1.0 / (pMapHits[i] + pMapMisses[i]);
        double ratio_threshold = 0.3;

        if (ratio > ratio_threshold) {
            pMap[i] = 100;
        } else if (ratio <= ratio_threshold) {
            pMap[i] = 0;
        }
    }
#endif //PROBLEM_2

#ifdef PROBLEM_3
    // BFS查找边界
    pMapIsAccessed = new int[mapParams.width * mapParams.height];
    std::vector<std::pair<int, int>> directions = { { 1, 0 }, { 0, 1 }, { -1, 0 }, { 0, -1 } };
    queue<GridIndex> q;
    q.push(GridIndex(0, 0));

    while (!q.empty()) {
        int size = q.size();
        for (size_t i = 0; i < size; i++) {
            GridIndex node = q.front();
            q.pop();
            for (size_t k = 0; k < 4; k++) {
                pair<int, int> dir = directions[k];
                int tmpx = node.x + dir.first;
                int tmpy = node.y + dir.second;
                int linearIndex = tmpy + tmpx * mapParams.width;
                int centerIndex = node.y + node.x * mapParams.width;

                if (tmpx >= 0 && tmpx < mapParams.height && tmpy >= 0 && tmpy < mapParams.width && pMapIsAccessed[linearIndex] == 0) {
                    // 对于边界点的处理
                    if (pMapTSDF[linearIndex] * pMapTSDF[centerIndex] < 0) {
                        // 选择pMapTSDF绝对值较小的一个栅格，认为是障碍物所在的栅格
                        if (abs(pMapTSDF[linearIndex]) < abs(pMapTSDF[centerIndex])) {
                            pMap[linearIndex] = 100;
                        } else {
                            pMap[centerIndex] = 100;
                        }
                    }
                    // 访问过的点不再进行访问
                    pMapIsAccessed[linearIndex] = 1;
                    q.push(GridIndex(tmpx, tmpy));
                }
            }
        }
    }
#endif //PROBLEM_3

    //end of TODO
    std::cout << "建图完毕" << std::endl;
}

//发布地图．
void PublishMap(ros::Publisher& map_pub)
{
    nav_msgs::OccupancyGrid rosMap;

    rosMap.info.resolution = mapParams.resolution;
    rosMap.info.origin.position.x = 0.0;
    rosMap.info.origin.position.y = 0.0;
    rosMap.info.origin.position.z = 0.0;
    rosMap.info.origin.orientation.x = 0.0;
    rosMap.info.origin.orientation.y = 0.0;
    rosMap.info.origin.orientation.z = 0.0;
    rosMap.info.origin.orientation.w = 1.0;

    rosMap.info.origin.position.x = mapParams.origin_x;
    rosMap.info.origin.position.y = mapParams.origin_y;
    rosMap.info.width = mapParams.width;
    rosMap.info.height = mapParams.height;
    rosMap.data.resize(rosMap.info.width * rosMap.info.height);

    for (int i = 0; i < mapParams.width * mapParams.height; i++) {
        if (pMap[i] == 50) {
            rosMap.data[i] = -1;
        } else {
            rosMap.data[i] = pMap[i];
        }
    }

    // for (int i = 0; i < mapParams.width * mapParams.height; i++) {
    //     if (pMap[i] == 50) {
    //         rosMap.data[i] = -1.0;
    //     } else if (pMap[i] < 50) {
    //         rosMap.data[i] = 0;
    //     } else if (pMap[i] > 50) {
    //         rosMap.data[i] = 100;
    //     }
    // }

    rosMap.header.stamp = ros::Time::now();
    rosMap.header.frame_id = "map";

    map_pub.publish(rosMap);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "OccupanyMapping");

    ros::NodeHandle nodeHandler;

    ros::Publisher mapPub = nodeHandler.advertise<nav_msgs::OccupancyGrid>("laser_map", 1, true);

    std::vector<Eigen::Vector3d> robotPoses;
    std::vector<GeneralLaserScan> generalLaserScans;

    std::string basePath = "/home/sunm/work/code/shenlan-2d/7/HW7/OccupanyMappingProject/src/data";

    std::string posePath = basePath + "/pose.txt";
    std::string anglePath = basePath + "/scanAngles.txt";
    std::string scanPath = basePath + "/ranges.txt";

    //读取数据
    ReadPoseInformation(posePath, robotPoses);

    ReadLaserScanInformation(anglePath,
        scanPath,
        generalLaserScans);

    //设置地图信息
    SetMapParams();

    OccupanyMapping(generalLaserScans, robotPoses);

    PublishMap(mapPub);

    ros::spin();

    DestoryMap();

    std::cout << "Release Memory!!" << std::endl;
}
