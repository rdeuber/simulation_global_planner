#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <tf/tf.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

using std::string;
 
#ifndef SIMULATION_GLOBAL_PLANNER_CPP
#define SIMULATION_GLOBAL_PLANNER_CPP

namespace simulation_global_planner {

	class SimulationGlobalPlanner : public nav_core::BaseGlobalPlanner {
		public:
			SimulationGlobalPlanner();
			SimulationGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
			 
			/** overridden classes from interface nav_core::BaseGlobalPlanner **/
			void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
			bool makePlan(const geometry_msgs::PoseStamped& start,
				const geometry_msgs::PoseStamped& goal,
				std::vector<geometry_msgs::PoseStamped>& plan);
	};
};
#endif
 
