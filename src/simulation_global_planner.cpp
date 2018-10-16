#include <pluginlib/class_list_macros.h>
#include <simulation_global_planner/simulation_global_planner.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(simulation_global_planner::SimulationGlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace simulation_global_planner {

SimulationGlobalPlanner::SimulationGlobalPlanner (){

}

SimulationGlobalPlanner::SimulationGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
   initialize(name, costmap_ros);
}


void SimulationGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
  ROS_INFO("SimulationGlobalPlanner initialization worked!");
}

bool SimulationGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
  ROS_INFO("Size: %d", plan.size() );
  plan.push_back(start);
  for (int i=0; i<20; i++){
    geometry_msgs::PoseStamped new_goal = goal;

    double angle = atan2((goal.pose.position.y-start.pose.position.y), (goal.pose.position.x-start.pose.position.x));
    tf::Quaternion goal_quat = tf::createQuaternionFromYaw(angle);

    new_goal.pose.position.x=(goal.pose.position.x-start.pose.position.x)*(i+1.0)/20 + start.pose.position.x;
    new_goal.pose.position.y=(goal.pose.position.y-start.pose.position.y)*(i+1.0)/20 + start.pose.position.y;


    new_goal.pose.orientation.x = goal_quat.x();
    new_goal.pose.orientation.y = goal_quat.y();
    new_goal.pose.orientation.z = goal_quat.z();
    new_goal.pose.orientation.w = goal_quat.w();

    plan.push_back(new_goal);
   }
   plan.push_back(goal);
   
  return true;
 }
};