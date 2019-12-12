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
/**************************************************************************
 * Desc: Global map (grid-based)
 * Author: Andrew Howard
 * Date: 6 Feb 2003
 * CVS: $Id: map.h 1713 2003-08-23 04:03:43Z inspectorg $
 **************************************************************************/

#ifndef MAP_H
#define MAP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Forward declarations
struct _rtk_fig_t;

// Limits
#define MAP_WIFI_MAX_LEVELS 8
#define CELL_STATUS_FREE (-1)
#define CELL_STATUS_UNKNOWN (0)
#define CELL_STATUS_OCC (+1)

// Description for a single map cell.
/**
 *单独的地图cell类型
 */
typedef struct
{
    // Occupancy state (-1 = free, 0 = unknown, +1 = occ)
    int occ_state;

    // Distance to the nearest occupied cell
    double occ_dist;

    double score;

    // Wifi levels
    //int wifi_levels[MAP_WIFI_MAX_LEVELS];

} map_cell_t;

// Description for a map
/**
 *地图的数据结构
 */
typedef struct
{
    // Map origin; the map is a viewport onto a conceptual larger map.
    double origin_x, origin_y;

    // Map scale (m/cell) 地图的分辨率
    double resolution;

    // Map dimensions (number of cells) X Y方向的栅格数
    int size_x, size_y;

    // The map data, stored as a grid
    map_cell_t* cells;

    // Max distance at which we care about obstacles, for constructing
    // likelihood field
    double max_occ_dist; //在似然场模型中，障碍物影响的最大距离

    double min_score;

    double likelihood_sigma; //似然场的标准差

} map_t;

/**************************************************************************
 * Basic map functions
 **************************************************************************/

// Create a new (empty) map 创建一个空地图
map_t* map_alloc(void);

// Destroy a map    释放地图的内存
void map_free(map_t* map);

// Get the cell at the given point 返回某一个地图栅格
map_cell_t* map_get_cell(map_t* map, double ox, double oy, double oa);

// Update the cspace distances 更新地图的似然场模型中的最大影响距离
void map_update_cspace(map_t* map, double max_occ_dist);

/**************************************************************************
 * Map manipulation macros
 **************************************************************************/

// Convert from map index to world coords  地图坐标转换到世界坐标
#define MAP_WXGX(map, i) (map->origin_x + ((i)-map->size_x / 2) * map->resolution)
#define MAP_WYGY(map, j) (map->origin_y + ((j)-map->size_y / 2) * map->resolution)

// Convert from world coords to map coords 世界坐标转换到地图坐标
#define MAP_GXWX(map, x) (floor((x - map->origin_x) / map->resolution + 0.5) + map->size_x / 2)
#define MAP_GYWY(map, y) (floor((y - map->origin_y) / map->resolution + 0.5) + map->size_y / 2)

#define MAP_GXWX_DOUBLE(map, x) ((x - map->origin_x) / map->resolution + 0.5 + double(map->size_x / 2))
#define MAP_GYWY_DOUBLE(map, y) ((y - map->origin_y) / map->resolution + 0.5 + double(map->size_y / 2))

// Test to see if the given map coords lie within the absolute map bounds. 判断是否出界
#define MAP_VALID(map, i, j) ((i >= 0) && (i < map->size_x) && (j >= 0) && (j < map->size_y))

// Compute the cell index for the given map coords. 把地图坐标转化为Index
#define MAP_INDEX(map, i, j) ((i) + (j)*map->size_x)

#ifdef __cplusplus
}
#endif

#endif
