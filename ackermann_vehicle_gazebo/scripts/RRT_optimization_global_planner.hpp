/** include the libraries you need in your planner here */
/** for global path planner interface */
#ifndef RRT_OPT
#define RRT_OPT
#include "kd-tree.hpp"
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <tf/tf.h>
#include <algorithm>
#include <chrono>
#include <vector>
#include <nlopt.hpp>
#include <iomanip>
#include <math.h>
#include <limits>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <set>
using std::string;

namespace rrt_planner
{
    struct Bubble
    {
        tf::Vector3 pose;
        float radius;
        float theta = 0.0;
        Bubble(){
            this->pose = tf::Vector3();
            this->radius = 0.0;
        }
        Bubble(tf::Vector3 pos, float radius)
        {
            this->pose = pos;
            this->radius = radius;
        }
        Bubble(tf::Vector3 pos, float radius, float theta)
        {
            this->pose = pos;
            this->radius = radius;
            this->theta = theta;
        }
    };
    
    struct State
    {
        tf::Vector3 pose;
        float theta;
        State *parent = NULL;
        float cost = 0.0;
        State(tf::Vector3 pose, float theta)
        {
            this->pose = pose;
            this->theta = theta;
        }
        State(tf::Vector3 pose, float theta, State *parent)
        {
            this->pose = pose;
            this->theta = theta;
            this->parent = parent;
        }
        State(tf::Vector3 pose, float theta, State *parent, float cost)
        {
            this->pose = pose;
            this->theta = theta;
            this->parent = parent;
            this->cost = cost;
        }
        State()
        {
            this->pose = tf::Vector3();
            this->theta = 0.0;
        }

        friend State operator+(State const &obj1, State const &obj2);

        void add(State const &obj1)
        {
            this->pose += obj1.pose;
            this->theta += obj1.theta;
        }
    };
    State operator+(State const &obj1, State const &obj2)
    {
        return State(obj1.pose + obj2.pose, obj1.theta + obj2.theta);
    }

    State *operator+(State *obj1, State obj2)
    {
        return new State(obj1->pose + obj2.pose, obj1->theta + obj2.theta);
    }

    //используется для представления дискретного состояния
    struct discreteState {
        unsigned int x = 0;
        unsigned int y = 0;
        unsigned int theta = 0;
        mutable int value = 0; // 0 - не использована, 1 - занята но может быть выполнен castState, 2 - полностью занята  
 
        discreteState(float x, float y, float theta, int val = 0) 
        {
            this->x = x;
            this->y = y;
            this->theta = theta;
            this->value = val;
        }

        void setValue(unsigned int val)
        {
            this->value = val;
        }

        friend bool cmpD(const discreteState* l, const discreteState* r);
    };

    struct continuousState {
        float x;
        float y; 
        float theta;
        float cost;
        float hcost;
        continuousState* parent;

        continuousState(){
            this->x = 0.0;
            this->y = 0.0;
            this->theta = 0.0;
            this->cost = 0.0;
            this->hcost = 0.0;
            this->parent = nullptr;
        }

        continuousState(float x, float y, float theta, continuousState* parent = nullptr, float cost = 0.0, float hcost = 0.0){
            this->x = x;
            this->y = y;
            this->theta = theta;
            this->cost = cost;
            this->hcost = hcost;
            this->parent = parent;
        }

        void setCost(float cost)
        {
            this->cost = cost;
        }

        void setHCost(float hcost)
        {
            this->hcost = hcost;
        }

        void setParent(continuousState* parent)
        {
            this->parent = parent;
        }
    }; 

    struct CompareC
    {
        bool operator ()(const rrt_planner::continuousState* const & l, const rrt_planner::continuousState* const & r) const
        { 
            if(l->hcost > r->hcost)
            {
                return true;
            }
            return false;
        }
    };

    struct CompareD
    {
        bool operator ()(const rrt_planner::discreteState* const &l, const rrt_planner::discreteState* const & r) const
        { 
            if(l->x < r->x)
            {
                return true;
            }
            if(l->x > r->x)
            {
                return false;
            }
            if(l->y < r->y)
            {
                return true;
            }
            if(l->y > r->y)
            {
                return false;
            }
            if(l->theta < r->theta)
            {
                return true;
            }
            if(l->theta > r->theta)
            {
                return false;
            }
            return false;
        }
    };

    float computeEuclidian(continuousState* state)
    {
        if(state->parent == nullptr)
        {
            return 0.0;
        }
        float x = state->x - state->parent->x;
        float y = state->y - state->parent->y;
        return sqrt(x * x + y * y);
    }; 

    float computeHeuristic(continuousState* state, tf::Vector3& goal)
    {
        float x = goal.x() - state->x;
        float y = goal.y() - state->y;
        return sqrt(x * x + y * y);
    };

    class AStarInterface
    {
        private:
            costmap_2d::Costmap2D* map;
            unsigned int scale;
            unsigned int thetaParts;
            tf::Vector3 goal;
        public:
            std::set<discreteState*, CompareD> closed;
            std::priority_queue<continuousState*, std::vector<continuousState*>, CompareC> opened;
            //scale - ширина клетки
            AStarInterface(costmap_2d::Costmap2D* map, tf::Vector3 goal, unsigned int scale = 1, unsigned int thetaParts = 18)
            {
                this->map = map;
                while(this->map->getSizeInCellsX() % scale != 0)
                {
                    scale--;
                }
                this->scale = scale;
                this->thetaParts = thetaParts;
                this->goal = goal;
            };

            continuousState* createCState(tf::Vector3& pose)
            {
                return(new continuousState(pose.x(), pose.y(), pose.z()));
            };

            continuousState* createCState(tf::Vector3& pose, continuousState* parent)
            {
                return(new continuousState(pose.x(), pose.y(), pose.z(), parent));
            };            

            discreteState* createDState(continuousState* state, int val = 0)
            {
                unsigned int x, y;
                this->map->worldToMap(state->x, state->y, x, y);
                float angle = fmod(angle, M_PI * 2);
                if(angle < 0.0)
                {
                    angle = M_PI * 2 + angle;
                }
                float onePart = M_PI * 2 / (float)this->thetaParts;
                int part = angle / onePart;
                return new discreteState(x / this->scale, y / this->scale, part, val);
            };

            bool collisionCheck(float x, float y)
            {
                unsigned int mx, my;
                this->map->worldToMap(x, y, mx, my);
                if(this->map->getCost(mx, my) > 127)
                {
                    return true;
                } else
                {
                    return false;
                }
            };

            void pushOpen(continuousState* state, float mul = 1.0)
            {
                state->cost = state->parent->cost + computeEuclidian(state) * mul;
                state->hcost = state->cost + computeHeuristic(state, this->goal);
                state->theta += 0.005;
                this->opened.push(state);
            };
            

            continuousState* popOpen()
            {
                continuousState* temp = this->opened.top();
                this->opened.pop();
                return temp;

            };

            discreteState* getClosed(discreteState* state)
            {
                auto search = this->closed.find(state);
                if (search != this->closed.end()) 
                {
                    return (*search);
                } else 
                {
                    this->closed.insert(state);
                    search = this->closed.find(state);
                    return (*search);
                }
            };

            discreteState* getClosed(continuousState* state)
            {
                discreteState* temp = this->createDState(state);
                auto search = this->closed.find(temp);
                if (search != this->closed.end()) 
                {
                    return (*search);
                } else 
                {
                    this->closed.insert(temp);
                    search = this->closed.find(temp);
                    return (*search);
                }
            };

            void pushClosed(continuousState* state, int val)
            {
                this->closed.insert(this->createDState(state));
            };

            std::vector<State*> buildStatePath(continuousState* head)
            {
                std::vector<State*> path;
                while(head != nullptr)
                {
                    path.push_back(new State({head->x, head->y, 0.0}, head->theta));
                    head = head->parent;
                }
                std::reverse(std::begin(path), std::end(path));
                return path;
            }

            float countLength(continuousState* head)
            {
                float pathLength = 0.0;
                while(head->parent != nullptr)
                {
                    pathLength += computeEuclidian(head);
                    head = head->parent;
                }
                return pathLength;
            }

            std::vector<geometry_msgs::PoseStamped> buildPath(continuousState* head, std_msgs::Header header)
            {
                std::vector<continuousState*> path;
                path.push_back(new continuousState(this->goal.x(), this->goal.y(), this->goal.z()));
                while(head != nullptr)
                {
                    path.push_back(head);
                    head = head->parent;
                }
                std::reverse(std::begin(path), std::end(path));
                std::vector<geometry_msgs::PoseStamped> gPath;
                for(continuousState*& poi : path)
                {
                    geometry_msgs::PoseStamped msgs;
                    msgs.header = header;
                    tf::Quaternion q;
                    q.setEuler(0, 0, poi->theta);
                    tf::quaternionTFToMsg(q, msgs.pose.orientation);
                    msgs.pose.position.x = poi->x;
                    msgs.pose.position.y = poi->y;
                    msgs.pose.position.z = 0.0;
                    gPath.push_back(msgs);
                }
                return gPath;
            };

            continuousState* stateCast(State* state, continuousState* parent = nullptr, float cost = 0.0, float hcost = 0.0)
            {
                return new continuousState(state->pose.x(), state->pose.y(), state->theta, parent, cost, hcost);
            };
    };

    

    class GlobalPlanner : public nav_core::BaseGlobalPlanner
    {
    public:
        visualization_msgs::MarkerArray ma;
        ros::NodeHandle nh;
        ros::Publisher publisher;
        float zpose = 0.2;
        float maxSteeringAngle = 0.4; // of imgainary middle wheel in rads
        float deltaTime = 2.0;
        float carLength = 0.4 * 3.4 * 0.7;
        float velocity = 0.5;
        float lowerBound = this->deltaTime * this->velocity / 2.0; 
        float upperBound = this->deltaTime * this->velocity;
        double startYaw;
        double goalYaw;
        geometry_msgs::PoseStamped stamp;
        std_msgs::Header header;
        costmap_2d::Costmap2DROS *map;
        GlobalPlanner();
        GlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

        /** overridden classes from interface nav_core::BaseGlobalPlanner **/
        void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
        visualization_msgs::Marker createVizCylinder(std::pair<float, float> pos, float radius, int id);
        std::vector<geometry_msgs::PoseStamped> createRRTPath(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal);
        tf::Vector3 genSample(const std::vector<tf::Vector3> &constraints, int id);
        State * genState(State *closestNode, tf::Vector3 sample);
        State *findNearestNode(kd_tree::KDTree<State *> &tree, const tf::Vector3 sample);
        State * solveRK(State *pose, float steeringAngle, float vel = 0.5);
        void bubbleTubeCreator(std::vector<State *>& path, std::vector<Bubble *>& tube);
        Bubble* generateBubble(State * point, float lastBubbleRadius);
        bool bubbleCollisionCheck(Bubble * point, tf::Vector3& obstacleDir, float radius);
        visualization_msgs::Marker createArrow(tf::Vector3 start, int id);
        visualization_msgs::Marker createVizPoint(std::pair<float, float> pos, int id);
        tf::Vector3 getKinematicsDerivatives(State *pose, float steeringAngle, float vel);
        Bubble* translateBubble(State* point);
        visualization_msgs::Marker createLine(std::pair<float, float> start, std::pair<float, float> end, int id);
        void convexOptimization(std::vector<Bubble *> &listOfBubbles, std::vector<double> &optPos);
        bool collisionCheck(tf::Vector3 pos, unsigned char targetCost = 0);
        bool checkReedShepp(State * state, const geometry_msgs::PoseStamped goal);
        bool makePlan(const geometry_msgs::PoseStamped &start,
                      const geometry_msgs::PoseStamped &goal,
                      std::vector<geometry_msgs::PoseStamped> &plan);
    };
}; // namespace rrt_planner
#endif