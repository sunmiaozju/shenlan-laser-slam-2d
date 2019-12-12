/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "map.h"
#include <math.h>
#include <queue>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

class CellData {
public:
    map_t* map_;
    unsigned int i_, j_;
    unsigned int src_i_, src_j_;
};

/*
 * 这里的CachedDistanceMap实际上是一个以(0,0)为中心的局部距离地图
 * 这个地图的每一个点都存储着该点的坐标到(0,0)的欧几里得距离。
 * 这里的CacheDistanceMap是用来计算栅格离障碍物距离的时候减少计算量的。
 * 相当于开始进行一次计算的话，后面只要查表就可以计算出来距离，而不需要反复的调用sqrt来求解
*/
class CachedDistanceMap {
public:
    CachedDistanceMap(double scale, double max_dist)
        : distances_(NULL)
        , scale_(scale)
        , max_dist_(max_dist)
    {
        //最大距离对应的cell的个数
        cell_radius_ = max_dist / scale;

        distances_ = new double*[cell_radius_ + 2];
        for (int i = 0; i <= cell_radius_ + 1; i++) {
            distances_[i] = new double[cell_radius_ + 2];
            for (int j = 0; j <= cell_radius_ + 1; j++) {
                distances_[i][j] = sqrt(i * i + j * j);
            }
        }
    }

    ~CachedDistanceMap()
    {
        if (distances_) {
            for (int i = 0; i <= cell_radius_ + 1; i++)
                delete[] distances_[i];
            delete[] distances_;
        }
    }
    double** distances_;
    double scale_;
    double max_dist_;
    int cell_radius_;
};

bool operator<(const CellData& a, const CellData& b)
{
    return a.map_->cells[MAP_INDEX(a.map_, a.i_, a.j_)].occ_dist > a.map_->cells[MAP_INDEX(b.map_, b.i_, b.j_)].occ_dist;
}

CachedDistanceMap* get_distance_map(double scale, double max_dist)
{
    static CachedDistanceMap* cdm = NULL;

    if (!cdm || (cdm->scale_ != scale) || (cdm->max_dist_ != max_dist)) {
        if (cdm)
            delete cdm;
        cdm = new CachedDistanceMap(scale, max_dist);
    }

    return cdm;
}

//计算到障碍物的距离
/**
 * @brief enqueue
 * @param map       对应的地图
 * @param i         该点对应的x坐标
 * @param j         该点对应的y坐标
 * @param src_i     该点对应的障碍物的x坐标
 * @param src_j     该点对应的障碍物的y坐标
 * @param Q
 * @param cdm
 * @param marked
 */
void enqueue(map_t* map, unsigned int i, unsigned int j,
    unsigned int src_i, unsigned int src_j,
    std::priority_queue<CellData>& Q,
    CachedDistanceMap* cdm,
    unsigned char* marked)
{
    //如果已经被计算过了 则直接返回
    if (marked[MAP_INDEX(map, i, j)])
        return;

    //这里的距离是栅格的距离
    unsigned int di = abs(i - src_i);
    unsigned int dj = abs(j - src_j);
    double distance = cdm->distances_[di][dj];

    if (distance > cdm->cell_radius_)
        return;

    //转换为实际距离
    map->cells[MAP_INDEX(map, i, j)].occ_dist = distance * map->resolution;

    double z = map->cells[MAP_INDEX(map, i, j)].occ_dist;
    map->cells[MAP_INDEX(map, i, j)].score = exp(-(z * z) / (2 * map->likelihood_sigma * map->likelihood_sigma));

    CellData cell;
    cell.map_ = map;
    cell.i_ = i;
    cell.j_ = j;
    cell.src_i_ = src_i;
    cell.src_j_ = src_j;

    Q.push(cell);

    marked[MAP_INDEX(map, i, j)] = 1;
}

// Update the cspace distance values

/**
 * @brief map_update_cspace
 * 更新地图的距离值 这个函数用来计算用来定位的地图中的每隔栅格到最近障碍物的距离
 * 其中障碍物的栅格的距离为0 然后通过bfs进行搜索来计算每一个栅格到障碍物的距离
 * @param map
 * @param max_occ_dist
 */
void map_update_cspace(map_t* map, double max_occ_dist)
{
    unsigned char* marked;
    std::priority_queue<CellData> Q;

    marked = new unsigned char[map->size_x * map->size_y];
    memset(marked, 0, sizeof(unsigned char) * map->size_x * map->size_y);

    map->max_occ_dist = max_occ_dist;

    //得到一个CachedDistanceMap
    CachedDistanceMap* cdm = get_distance_map(map->resolution, map->max_occ_dist);

    //这个sigma已经在外面设置过了 在handmapmsg里面就会设置
    map->min_score = exp(-max_occ_dist * max_occ_dist / (2 * map->likelihood_sigma * map->likelihood_sigma));

    // Enqueue all the obstacle cells
    // 所有的障碍物都放入队列中
    CellData cell;
    cell.map_ = map;

    //计算出来所有的边界障碍物 只有边界障碍物才用来进行匹配 其他的障碍物都当成no-information

    /*所有障碍物的栅格  离障碍物的距离都标志为0  非障碍物的栅格都标记为max_occ_dist*/
    for (int i = 0; i < map->size_x; i++) {
        cell.src_i_ = cell.i_ = i;
        for (int j = 0; j < map->size_y; j++) {
            if (map->cells[MAP_INDEX(map, i, j)].occ_state == CELL_STATUS_OCC) {
                map->cells[MAP_INDEX(map, i, j)].occ_dist = 0.0;
                map->cells[MAP_INDEX(map, i, j)].score = 1.0;
                cell.src_j_ = cell.j_ = j;
                marked[MAP_INDEX(map, i, j)] = 1;
                Q.push(cell);
            } else
                map->cells[MAP_INDEX(map, i, j)].occ_dist = max_occ_dist;
        }
    }

    while (!Q.empty()) {
        CellData current_cell = Q.top();

        /*往上、下、左、右四个方向拓展*/
        if (current_cell.i_ > 0)
            enqueue(map, current_cell.i_ - 1, current_cell.j_,
                current_cell.src_i_, current_cell.src_j_,
                Q, cdm, marked);

        if (current_cell.j_ > 0)
            enqueue(map, current_cell.i_, current_cell.j_ - 1,
                current_cell.src_i_, current_cell.src_j_,
                Q, cdm, marked);

        if ((int)current_cell.i_ < map->size_x - 1)
            enqueue(map, current_cell.i_ + 1, current_cell.j_,
                current_cell.src_i_, current_cell.src_j_,
                Q, cdm, marked);

        if ((int)current_cell.j_ < map->size_y - 1)
            enqueue(map, current_cell.i_, current_cell.j_ + 1,
                current_cell.src_i_, current_cell.src_j_,
                Q, cdm, marked);

        Q.pop();
    }

    delete[] marked;
}
