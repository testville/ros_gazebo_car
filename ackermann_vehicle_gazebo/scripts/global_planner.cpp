#include <pluginlib/class_list_macros.h>
#include "global_planner.h"
#include <tf/tf.h>
#include <ros/console.h>
#include <algorithm>
#include <chrono>
#include "std_msgs/String.h"
#include <sstream>
#include "kd-tree.hpp"
//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace rrt_global_planner
{

Node::Node(Node *prevNode, std::pair<float, float> state, float cost, float steerAngle)
{
  this->prevNode = prevNode;
  this->state = state;
  this->cost = cost;
}

GlobalPlanner::GlobalPlanner()
{
}

GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
  initialize(name, costmap_ros);
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
  this->costmap2d_ros = costmap_ros;
  this->getCostMap();
  this->nh = ros::NodeHandle("~");
  this->publisher = this->nh.advertise<visualization_msgs::MarkerArray>("/vizmrks", 100);
}

void GlobalPlanner::getCostMap()
{
  this->costmap2d = this->costmap2d_ros->getCostmap();
}

float GlobalPlanner::distance(std::pair<float, float> start, std::pair<float, float> goal)
{
  float a = goal.first - start.first;
  float b = goal.second - start.second;
  return (sqrt(a * a + b * b));
}

visualization_msgs::Marker GlobalPlanner::createLine(std::pair<float, float> start, std::pair<float, float> end, int id)
{
  visualization_msgs::Marker line;
  line.header.frame_id = "/map";
  line.header.stamp = ros::Time::now();
  line.ns = "lines";
  line.action = visualization_msgs::Marker::ADD;
  line.pose.orientation.w = 1.0;
  line.id = id;
  line.type = visualization_msgs::Marker::LINE_STRIP;
  line.scale.x = 0.02;
  line.color.r = 1.0f;
  line.color.a = 1.0;
  geometry_msgs::Point lineStart, lineEnd;
  lineStart.x = start.first;
  lineStart.y = start.second;
  lineStart.z = this->stamp.pose.position.z;
  lineEnd.x = end.first;
  lineEnd.y = end.second;
  lineEnd.z = this->stamp.pose.position.z;
  line.points.push_back(lineStart);
  line.points.push_back(lineEnd);
  return line;
}

visualization_msgs::Marker GlobalPlanner::createVizPoint(std::pair<float, float> &pos, int id)
{
  visualization_msgs::Marker point;
  point.header.frame_id = "/map";
  point.header.stamp = ros::Time::now();
  point.ns = "points";
  point.action = visualization_msgs::Marker::ADD;
  point.pose.orientation.w = 1.0;
  point.id = id;
  point.type = visualization_msgs::Marker::SPHERE;
  point.scale.x = 0.1;
  point.scale.y = 0.1;
  point.scale.z = 0.1;
  point.color.r = 1.0f;
  point.color.a = 1.0;
  geometry_msgs::Pose p;
  p.position.x = pos.first;
  p.position.y = pos.second;
  p.position.z = this->stamp.pose.position.z;
  p.orientation = this->stamp.pose.orientation;
  point.pose = p;
  return point;
}

visualization_msgs::Marker GlobalPlanner::createVizPointE(std::pair<float, float> &pos, int id)
{
  visualization_msgs::Marker point;
  point.header.frame_id = "/map";
  point.header.stamp = ros::Time::now();
  point.ns = "pointsCircle";
  point.action = visualization_msgs::Marker::ADD;
  point.pose.orientation.w = 1.0;
  point.id = id;
  point.type = visualization_msgs::Marker::SPHERE;
  point.scale.x = 0.1;
  point.scale.y = 0.1;
  point.scale.z = 0.1;
  point.color.b = 1.0f;
  point.color.a = 1.0;
  geometry_msgs::Pose p;
  p.position.x = pos.first;
  p.position.y = pos.second;
  p.position.z = this->stamp.pose.position.z;
  p.orientation = this->stamp.pose.orientation;
  point.pose = p;
  return point;
}

visualization_msgs::Marker GlobalPlanner::createArrow(std::pair<float, float> &start, std::pair<float, float> &end, int id)
{
  visualization_msgs::Marker arrow;
  arrow.header.frame_id = "/map";
  arrow.header.stamp = ros::Time::now();
  arrow.ns = "arrow";
  arrow.action = visualization_msgs::Marker::ADD;
  arrow.pose.orientation = this->stamp.pose.orientation;
  arrow.id = id;
  arrow.type = visualization_msgs::Marker::ARROW;
  arrow.scale.y = 0.05;
  arrow.scale.z = 0.05;
  arrow.color.b = 1.0f;
  arrow.color.a = 1.0;
  geometry_msgs::Point p1;
  geometry_msgs::Point p2;

  p1.x = start.first;
  p1.y = start.second;
  p1.z = this->stamp.pose.position.z;

  p2.x = end.first;
  p2.y = end.second;
  p2.z = this->stamp.pose.position.z;
  arrow.points.push_back(p1);
  arrow.points.push_back(p2);
  arrow.pose.position.x = start.first;
  arrow.pose.position.y = start.second;
  arrow.pose.position.z = this->stamp.pose.position.z;
  return arrow;
}

std::pair<float, float> GlobalPlanner::norm(std::pair<float, float> start, std::pair<float, float> goal)
{
  float a = goal.first - start.first;
  float b = goal.second - start.second;
  float d = this->distance(start, goal);
  return std::make_pair(a / d, b / d);
}

bool GlobalPlanner::collisionCheck(std::pair<float, float> start, std::pair<float, float> end, int numberOfSamples)
{
  std::pair<float, float> diff = make_pair((end.first - start.first) / (numberOfSamples + 1), (end.second - start.second) / (numberOfSamples + 1));
  std::pair<float, float> initSample = std::make_pair(start.first, start.second);
  initSample.first += diff.first;
  initSample.second += diff.second;
  ROS_INFO("-----start first: %f ; second: %f", start.first, start.second);
  for (int i = 0; i < numberOfSamples; i++)
  {
    unsigned int mx, my;
    this->costmap2d->worldToMap(initSample.first, initSample.second, mx, my);
    ROS_INFO("sample first: %f ; second: %f", initSample.first, initSample.second);
    unsigned char cost = this->costmap2d->getCost(mx, my);
    if (cost > 1)
    {
      return false;
    }
    initSample.first += diff.first;
    initSample.second += diff.second;
  }
  ROS_INFO("------end first: %f ; second: %f", end.first, end.second);
  return true;
}

void GlobalPlanner::pathOptimizer(Node *goal, float &cost, int numberOfSamples)
{
  ROS_INFO("asdasd");
  Node *potentialParent;
  Node *prev = goal->prevNode;
  Node *currentHead = goal;
  bool perfomImprovments = false;
  while (prev != NULL)
  {
    if (this->collisionCheck(currentHead->state, prev->state, numberOfSamples))
    {
      if (currentHead->prevNode != prev)
      {
        currentHead->prevNode = prev;
        currentHead->cost = prev->cost + this->distance(currentHead->state, prev->state);
        perfomImprovments = true;
      }
      prev = prev->prevNode;
    }
    else
    {
      currentHead = prev;
      prev = prev->prevNode;
    }
  }
  float costasd = 12471892;
  if (perfomImprovments)
  {
    this->pathOptimizer(goal, costasd, numberOfSamples);
  }
  else
  {
    return;
  }
}

std::vector<geometry_msgs::PoseStamped> GlobalPlanner::buildRRT(
    std::pair<float, float> start,
    std::pair<float, float> goal,
    int numberOfIter,
    float diffLength,
    float neighbourRadius,
    float epsilon,
    int numberOfSamples)
{
  std::vector<Node *> tree;

  tree.push_back(new Node(nullptr, start, 0, 0));
  kd_tree::KDTree<Node *> kdtree = kd_tree::KDTree<Node *>();
  kdtree.insert(start, new Node(nullptr, start, 0, 0));
  std::srand(unsigned(std::time(0)));
  ROS_INFO("start for loop in RRT");
  visualization_msgs::MarkerArray ma;
  bool pathFound = false;
  int noi = this->distance(start, goal) * 10;
  noi *= noi;
  Node *currentBestNode = NULL;
  float currentBestDistance = 100000000.0;

  if (noi > numberOfIter)
  {
    noi = numberOfIter;
  }
  int newCircle = 1;
  int circlePoint = 10000;
  for (int i = 0; i < noi; i++)
  {
    Node *tempState;
    float se_d = this->distance(start, goal);
    float srandom = start.first + se_d * 2 - se_d * 3 / 2;
    float signX = 1;
    float signY = 1;
    if (goal.first < start.first)
      signX = -1;
    if (goal.second < start.second)
      signY = -1;
    std::pair<float, float> randomState;
    if (currentBestNode == NULL)
    {
      randomState = std::make_pair(start.first + (2.5 * se_d * std::rand() / (float)RAND_MAX - se_d) * signX, start.second + (2.5 * se_d * std::rand() / (float)RAND_MAX - se_d) * signY);
    }
    else
    {
      float di = this->distance(start, goal);
      std::pair<float, float> dir = this->norm(start, goal);
      std::pair<float, float> center = std::make_pair(start.first + dir.first * (0.5 * di), start.second + dir.second * (0.5 * di));

      float thetaEllipse = acos(dir.first);
      if (dir.second < 0)
      {
        thetaEllipse *= -1;
      }
      float aAxis = currentBestDistance / 2.0; // тут нужно переписать a = currentBest - половина фокального расстояния
      float c = di / 2.0;
      float bAxis = sqrt(aAxis * aAxis - c * c);
      float ro = std::rand() / (float)RAND_MAX;
      float phi = std::rand() / (float)RAND_MAX * 2 * 3.1415;
      float xRandom = sqrt(ro) * cos(phi) * aAxis;
      float yRandom = sqrt(ro) * sin(phi) * bAxis;
      float xRandom2 = xRandom * cos(thetaEllipse) - yRandom * sin(thetaEllipse);
      float yRandom2 = yRandom * cos(thetaEllipse) + xRandom * sin(thetaEllipse);
      randomState = std::make_pair(xRandom2 + center.first, yRandom2 + center.second);
    }
    float min = 100000.0;
    kd_tree::KDNode<Node *> *n = NULL;
    kdtree.NNSearch(randomState, n);
    tempState = n->data;
    std::pair<float, float> normDiff = this->norm(tempState->state, randomState);
    std::pair<float, float> newPos = std::make_pair(tempState->state.first + normDiff.first * diffLength, tempState->state.second + normDiff.second * diffLength);
    unsigned int mx, my;
    this->costmap2d->worldToMap(newPos.first, newPos.second, mx, my);
    unsigned char cost = this->costmap2d->getCost(mx, my);
    if (cost > 127)
    {
      continue;
    }
    if (i % 100 == 0)
    {
      ROS_INFO("distance tempState newPos is %f", tempState->cost + this->distance(tempState->state, newPos));
    }
    Node *newNode = new Node(tempState, newPos, tempState->cost + this->distance(tempState->state, newPos), 0);
    //searching neighbours
    std::vector<kd_tree::KDNode<Node *> *> neighbours;
    kdtree.RangeSearch(std::make_pair<float>(newNode->state.first - neighbourRadius, newNode->state.first + neighbourRadius), std::make_pair<float>(newNode->state.second - neighbourRadius, newNode->state.second + neighbourRadius), neighbours);
    if (i % 100 == 0)
    {
      ROS_INFO("number of neighbours is %lu", neighbours.size());
    }
    //eval cost of newNode
    for (kd_tree::KDNode<Node *> *&node : neighbours)
    {
      if (node->data->cost + this->distance(node->data->state, newPos) < newNode->cost)
      {
        newNode->prevNode = node->data;
        newNode->cost = node->data->cost + this->distance(node->data->state, newNode->state);
      }
    }

    //revise cost of neighbours
    for (kd_tree::KDNode<Node *> *&node : neighbours)
    {
      if (node->data->cost > newNode->cost + this->distance(newNode->state, node->data->state))
      {
        node->data->prevNode = newNode;
        node->data->cost = newNode->cost + this->distance(newNode->state, node->data->state);
      }
    }
    tree.push_back(newNode);
    if (this->distance(newNode->state, goal) < epsilon)
    {
      if (currentBestNode == NULL)
      {
        currentBestNode = newNode;
        currentBestDistance = newNode->cost + this->distance(currentBestNode->state, goal);
      }
      else
      {
        if (currentBestNode->cost > newNode->cost)
        {
          currentBestNode = newNode;
          currentBestDistance = newNode->cost + this->distance(currentBestNode->state, goal);
          ROS_INFO("best posible distance is %f", this->distance(start, goal));
          ROS_INFO("current best distance is %f", currentBestDistance);
        }
      }
    }
    if (i == noi - 1)
    {
      if (currentBestNode == NULL)
      {
        if (noi < numberOfIter)
        {
          noi += 5;
        }
      }
    }

    kdtree.insert(newNode->state, newNode);

    //ma.markers.push_back(this->createArrow(newNode->prevNode->state, newNode->state, i));
    //ma.markers.push_back(this->createVizPoint(newNode->state, i));
    //ma.markers.push_back(this->createLine(newNode->prevNode->state, newNode->state, i));
  }
  //this->publisher.publish(ma);
  //build path
  float min = 10000000.0;
  Node *tempState = currentBestNode;

  float cost = 12312;
  //this->pathOptimizer(tempState, cost, 10);
  ROS_INFO("closes node %f", min);
  ROS_INFO("goal %f  %f", goal.first, goal.second);
  ROS_INFO("closest %f  %f", tempState->state.first, tempState->state.second);
  std::vector<geometry_msgs::PoseStamped> path = vector<geometry_msgs::PoseStamped>();
  int i = 0;
  tf::Quaternion prevQ = tf::Quaternion(this->stamp.pose.orientation.x, this->stamp.pose.orientation.y, this->stamp.pose.orientation.z, this->stamp.pose.orientation.w);
  while (true)
  {
    geometry_msgs::PoseStamped stamp = geometry_msgs::PoseStamped();
    stamp.header = this->stamp.header;
    stamp.pose.orientation = this->stamp.pose.orientation;
    geometry_msgs::Quaternion qm = geometry_msgs::Quaternion();
    stamp.pose.orientation.x = prevQ.x();
    stamp.pose.orientation.y = prevQ.y();
    stamp.pose.orientation.z = prevQ.z();
    stamp.pose.orientation.w = prevQ.w();
    if (tempState->prevNode != nullptr)
    {
      tf::Vector3 strt = tf::Vector3(tempState->prevNode->state.first, tempState->prevNode->state.second, this->stamp.pose.position.z);
      tf::Vector3 end = tf::Vector3(tempState->state.first, tempState->state.second, this->stamp.pose.position.z);
      tf::Vector3 dir = tf::Vector3(end - strt).normalize();
      float theta = acos(dir.x());
      if (dir.y() < 0)
      {
        theta *= -1;
      }
      prevQ = tf::Quaternion(0, 0, sin(theta / 2), cos(theta / 2));
      prevQ.normalize();
    }

    geometry_msgs::Point new_pose = geometry_msgs::Point();
    new_pose.x = tempState->state.first;
    new_pose.y = tempState->state.second;
    new_pose.z = this->stamp.pose.position.z;
    stamp.pose.position = new_pose;
    path.push_back(stamp);
    if (tempState->prevNode != nullptr)
    {
      tempState = tempState->prevNode;
    }
    else
    {
      break;
    }
  }
  std::reverse(std::begin(path), std::end(path));
  return path;
}

bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
{
  ROS_INFO("starting to create path!!!!@!@!");
  plan.clear();

  std::pair<float, float> startCell = std::make_pair(start.pose.position.x, start.pose.position.y);
  std::pair<float, float> goalCell = std::make_pair(goal.pose.position.x, goal.pose.position.y);
  this->stamp = goal;
  auto begin = std::chrono::high_resolution_clock::now();
  std::vector<geometry_msgs::PoseStamped> path = this->buildRRT(startCell, goalCell, 10000, 0.5, 0.6, 1, 10);
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<float> elapsed = finish - begin;
  ROS_INFO("comp time : %f", elapsed.count());
  plan = path;
  return true;
}
}; // namespace rrt_global_planner