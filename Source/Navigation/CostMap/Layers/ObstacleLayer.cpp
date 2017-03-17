/*
 * ObstacleLayer.cpp
 *
 *  Created on: 2017年3月11日
 *      Author: nico
 */

#include "ObstacleLayer.h"
#include "../Utils/Math.h"
#include <Parameter/Parameter.h>
#include <Service/Service.h>
#include <Service/ServiceType/RequestObstacle.h>
#include <Service/ServiceType/ResponseObstacle.h>
#include <Console/Console.h>
#include <Transform/DataTypes.h>

namespace NS_CostMap
{
ObstacleLayer::ObstacleLayer()
{

}

ObstacleLayer::~ObstacleLayer()
{

}

void ObstacleLayer::onInitialize()
{
	NS_NaviCommon::console.debug("obstacles layer is initializing!");
    NS_NaviCommon::Parameter parameter;
    parameter.loadConfigurationFile("obstacle_layer.xml");

    bool track_unknown_space;
    track_unknown_space = parameter.getParameter("track_unknown_space", layered_costmap_->isTrackingUnknown());
    if (track_unknown_space)
        default_value_ = NO_INFORMATION;
    else
        default_value_ = FREE_SPACE;

    ObstacleLayer::matchSize();
    active = true;

    double min_obstacle_height = parameter.getParameter("min_obstacle_height", (float)0.0);
    double max_obstacle_height = parameter.getParameter("max_obstacle_height", (float)2.0);
    bool inf_is_valid = parameter.getParameter("inf_is_valid", false);

    //回调
    obstacle_layer_loop = boost::thread(boost::bind(&ObstacleLayer::loopObstaclesMap, this));
}
void ObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                double* max_x, double* max_y)
{
	if (rolling_window_)
	    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
	if (!enabled_)
	    return;
	useExtraBounds(min_x, min_y, max_x, max_y);
	std::vector<NS_DataType::Point>::iterator it= obstacles_footprint_.begin();
	*min_x = *max_x = (*it).x;
	*min_y = *max_y = (*it).y;
	for (it = obstacles_footprint_.begin()+1; it != obstacles_footprint_.end(); it++)
	{
        *min_x = std::min((*it).x, *min_x);
        *min_y = std::min((*it).y, *min_y);
        *max_x = std::max((*it).x, *max_x);
        *max_y = std::max((*it).y, *max_y);
	}
	int index;
	bool lethal_start, lethal_end;
	int lethal_indStart;
	int lethal_indEnd;
    for (unsigned int y = origin_y_; y != size_y_; ++y) {
    	lethal_start = true;
    	lethal_indStart = lethal_indEnd = 0;
    	for(unsigned int x = origin_x_; x != size_x_; ++x) {
            index = getIndex(x,y);
            if (costmap_[index] == LETHAL_OBSTACLE) {
                if (lethal_start == true) {
                	lethal_start = false;
                	lethal_indStart = lethal_indEnd = index;  //默认开始和结束的障碍物是一样的
                } else { //找到一行中最后一个障碍物点
                    lethal_indEnd = index;
                }

            }
    	}
    	for (unsigned int p = lethal_indStart; p != lethal_indEnd; p++)
    	     costmap_[p] = LETHAL_OBSTACLE;
    }
}

void ObstacleLayer::updateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
	if (!enabled_)
	    return;

	if (footprint_clearing_enabled_)
	{
	    setConvexPolygonCost(transformed_footprint_, FREE_SPACE);
	}

	switch (combination_method_)
	{
	    case 0:  // Overwrite
	      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j); //更新的是矩形区域
	      break;
	    case 1:  // Maximum
	      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
	      break;
	    default:  // Nothing
	      break;
	}
}

void ObstacleLayer::activate()
{
	onInitialize();
}
void ObstacleLayer::deactivate()
{
	active = false;
	obstacle_layer_loop.join();
}
void ObstacleLayer::reset()
{
	deactivate();
	activate();
}


void ObstacleLayer::updateObstaclesFootPrint(std::vector<NS_DataType::Point> obstacles_footprint)
{
	if (obstacles_footprint.empty())
		return;

    obstacles_footprint_.clear();
	std::vector<NS_DataType::Point>::iterator it;
	for (it = obstacles_footprint.begin(); it != obstacles_footprint.end(); ++it)
		obstacles_footprint_.push_back(*it);
}

void ObstacleLayer::bresenhamOnObstacles(const NS_DataType::Point obstacle_p, const NS_DataType::Point obstacle_q)
{
	NS_DataType::Point begin_o = (obstacle_p.x < obstacle_q.x)? obstacle_p : obstacle_q;
	NS_DataType::Point end_o = (obstacle_p.x > obstacle_q.x)? obstacle_p: obstacle_q;
	int begin_mx = (int)((begin_o.x - origin_x_) / resolution_);  //world to map
	int begin_my = (int)((begin_o.y - origin_y_) / resolution_);
	if (begin_mx > size_x_ || begin_my > size_y_)
		return;
	int end_mx = (int)((end_o.x - origin_x_) / resolution_);
	int end_my = (int)((end_o.y - origin_y_) / resolution_);
	if (begin_mx > size_x_ || begin_my > size_y_)
			return;
	costmap_[getIndex(begin_mx, begin_my)] = LETHAL_OBSTACLE;  //设置起始障碍物
	costmap_[getIndex(end_mx, end_my)] = LETHAL_OBSTACLE;
	double k = (double)(end_my - begin_my) / (double)(end_mx - begin_mx);
	double b = (double)end_my - k*(double)end_mx;
	unsigned int index = 0;
	if (k > 0)  //求两个障碍物的连线
	{
		if (k <= 1) {
			for (unsigned int x = begin_mx+1, y = begin_my; x < end_mx; ++x) {
				double diff = x*k+b-y;
                if (diff < 0.5)
                	index = getIndex(x, y);
                else if (diff >= 0.5)
                	index = getIndex(x,++y);
                costmap_[index] = LETHAL_OBSTACLE;
			}
		} else if (k > 1) {
			for (unsigned int x = begin_mx, y = begin_my+1; y < end_my; ++y) {
				double diff = (y-b)/k-x;
			    if (diff < 0.5)
			        index = getIndex(x, y);
			    else if (diff >= 0.5)
			        index = getIndex(++x,y);
			    costmap_[index] = LETHAL_OBSTACLE;
		    }
		}
	} else { //矩阵Y轴向下的
		if (k >= -1) {
			for (unsigned int x = begin_mx+1, y = begin_my; x < end_mx; ++x) {
				double diff = y-(x*k+b);
		        if (diff < 0.5)
		            index = getIndex(x, y);
		        else if (diff >= 0.5)
		            index = getIndex(x,--y);
		        costmap_[index] = LETHAL_OBSTACLE;
			}
		} else if (k < -1) {
			for (unsigned int x = begin_mx, y = begin_my-1; y > end_my; --y) {
				double diff = (y-b)/k-x;
					if (diff < 0.5)
					    index = getIndex(x, y);
					else if (diff >= 0.5)
					    index = getIndex(++x,y);
					costmap_[index] = LETHAL_OBSTACLE;
			}
		}
	}
}

void ObstacleLayer::processObstacles(std::vector<NS_DataType::Point> obstacles_footprint)
{
	updateObstaclesFootPrint(obstacles_footprint);
	std::vector<NS_DataType::Point>::iterator it= obstacles_footprint_.begin();
	for (it = obstacles_footprint_.begin(); it != obstacles_footprint_.end()-1; it++)
	{
		bresenhamOnObstacles(*it, *(it+1));
	}
	bresenhamOnObstacles(*(obstacles_footprint_.end()-1), *(obstacles_footprint_.begin()));
}

void ObstacleLayer::loopObstaclesMap()
{
  NS_NaviCommon::Rate rate(map_update_frequency_);
  while(active)
  {
    NS_ServiceType::RequestObstacle req;
    NS_ServiceType::ResponseObstacle rep;
    service_->call(NS_NaviCommon::SERVICE_TYPE_OBSTACLES, &req, &rep);
    processObstacles(rep.obstacles);

    rate.sleep();
  }
}
}
