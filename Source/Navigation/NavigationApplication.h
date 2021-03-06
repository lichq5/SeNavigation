/*
 * NavigationApplication.h
 *
 *  Created on: 2016年12月3日
 *      Author: seeing
 */

#ifndef _NAVIGATIONAPPLICATION_H_
#define _NAVIGATIONAPPLICATION_H_

#include "../Application/Application.h"
#include "CostMap/CostmapWrapper.h"
#include <DataSet/DataType/PoseStamped.h>
#include "Planner/Base/GlobalPlannerBase.h"
#include "Planner/Base/LocalPlannerBase.h"
#include <boost/thread/thread.hpp>

namespace NS_Navigation {

enum MoveBaseState
{
  PLANNING,
  CONTROLLING,
  CLEARING
};

class NavigationApplication: public Application {
public:
  NavigationApplication();
  virtual ~NavigationApplication();
private:
  void loadParameters();

  void planLoop();
private:
  std::string global_planner_type_;
  std::string local_planner_type_;

private:
  //set up plan triple buffer
  std::vector<NS_DataType::PoseStamped>* global_planner_plan;
  std::vector<NS_DataType::PoseStamped>* latest_plan;
  std::vector<NS_DataType::PoseStamped>* local_planner_plan;

  NS_CostMap::CostmapWrapper* global_costmap;
  NS_CostMap::CostmapWrapper* local_costmap;

  NS_Planner::GlobalPlannerBase* global_planner;
  NS_Planner::LocalPlannerBase* local_planner;

  boost::thread plan_thread;
public:
  virtual void initialize();
  virtual void run();
  virtual void quit();
};

} /* namespace NS_Navigation */

#endif /* NAVIGATION_NAVIGATIONAPPLICATION_H_ */
