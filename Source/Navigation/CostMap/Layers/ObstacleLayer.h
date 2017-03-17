/*
 * ObstacleLayer.h
 *
 *  Created on: 2017年3月10日
 *      Author: nico
 */

#ifndef OBSTACLELAYER_H_
#define OBSTACLELAYER_H_

#include "../CostMap2D/CostMapLayer.h"
#include "../CostMap2D/LayeredCostMap.h"
#include <DataSet/DataType/OccupancyGrid.h>
#include <DataSet/DataType/OccupancyGridUpdate.h>
#include <DataSet/DataType/PointCloud.h>

#include <boost/thread/thread.hpp>

namespace NS_CostMap
{
class ObstacleLayer : public CostmapLayer
{
public:
    ObstacleLayer();
    virtual ~ObstacleLayer();
    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                double* max_x, double* max_y);
    virtual void updateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    virtual void activate();
    virtual void deactivate();
    virtual void reset();


protected:
    bool rolling_window_;
    bool footprint_clearing_enabled_;
    std::vector<NS_DataType::Point> transformed_footprint_;
    int combination_method_;
    std::vector<NS_DataType::Point> obstacles_footprint_;

private:
    boost::thread obstacle_layer_loop;
    bool map_update_frequency_;
    bool active;
    void updateObstaclesFootPrint(std::vector<NS_DataType::Point> obstacles_footprint);
    void bresenhamOnObstacles(const NS_DataType::Point obstacle_p, const NS_DataType::Point obstacle_q);
    void processObstacles(std::vector<NS_DataType::Point> obstacles_footprint);
    void loopObstaclesMap();
};
}
#endif /* OBSTACLELAYER_H_ */
