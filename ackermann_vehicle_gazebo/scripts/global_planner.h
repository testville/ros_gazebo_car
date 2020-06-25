/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <queue>
#include <cmath>

using std::string;

#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

namespace rrt_global_planner
{

class Node
{
public:
    Node *prevNode;
    std::pair<float, float> state;
    Node(Node *prevNode, std::pair<float, float> state, float cost, float steerAngle);
    float cost;
    float steerAngle;
};

class GlobalPlanner : public nav_core::BaseGlobalPlanner
{
private:
    ros::NodeHandle nh;
    ros::Publisher publisher;
    costmap_2d::Costmap2DROS *costmap2d_ros;
    costmap_2d::Costmap2D *costmap2d;
    void getCostMap();
    std::vector<geometry_msgs::PoseStamped> buildRRT(std::pair<float, float> startMapCell,
                                                     std::pair<float, float> goalMapCell,
                                                     int numberOfIter,
                                                     float diffLength,
                                                     float neighbourRadius,
                                                     float eps,
                                                     int numberOfSamples); //допустимая погрешность при нахождении цели
    float distance(std::pair<float, float> start, std::pair<float, float> goal);
    std::pair<float, float> norm(std::pair<float, float> start, std::pair<float, float> goal);
    geometry_msgs::PoseStamped stamp;
    bool pubMarkers = false;
    visualization_msgs::Marker createVizPoint(std::pair<float, float> &pos, int id);
    visualization_msgs::Marker createVizPointE(std::pair<float, float> &pos, int id);
    visualization_msgs::Marker createArrow(std::pair<float, float> &start, std::pair<float, float> &end, int id);
    visualization_msgs::Marker createLine(std::pair<float, float> start, std::pair<float, float> end, int id);
    bool collisionCheck(std::pair<float, float> start, std::pair<float, float> end, int numberOfSamples);
    void pathOptimizer(Node *goal, float &cost, int numberOfSamples);

public:
    GlobalPlanner();
    GlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

    /** overridden classes from interface nav_core::BaseGlobalPlanner **/
    void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped &start,
                  const geometry_msgs::PoseStamped &goal,
                  std::vector<geometry_msgs::PoseStamped> &plan);
};
}; // namespace rrt_global_planner
#endif
