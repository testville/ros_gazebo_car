//ЭТО ВСЕ ОБФУСКАЦИЯ ЕСЛИ ЧЕ
#include <pluginlib/class_list_macros.h>
#include "RRT_optimization_global_planner.hpp"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "utils.hpp"
#include <ompl-1.4/ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl-1.4/ompl/base/ScopedState.h>
#define ppos pose.position  //point position
#define pq pose.orientation //point quaternion
#define V3 tf::Vector3





PLUGINLIB_EXPORT_CLASS(rrt_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

namespace rrt_planner
{


    typedef struct constraints {
        int number = 0.0;
        int maxNumber = 0.0;
        Bubble* bubble = NULL;
        tf::Vector3 start, end;
    } dataConstraints;

    typedef struct initials {
        tf::Vector3 start, end;
    } dataInitial;


    double anglePenaltyFunction(tf::Vector3 v1, tf::Vector3 v2, tf::Vector3 v3)
    {
        tf::Vector3 vtemp1 = v2 - v1;
        tf::Vector3 vtemp2 = v3 - v2;
        double angle = abs(vtemp1.angle(vtemp2));
        if(angle > 0.35)
        {
            return 100000.0 * abs(tan(angle));
        } else {
            return abs(tan(angle)) * 1.0;
        }
    }

     double startAnglePenaltyFunction(double angle, tf::Vector3 v2, tf::Vector3 v3)
    {
        tf::Vector3 vtemp1 = tf::Vector3({cos(angle), sin(angle), 0.0});
        tf::Vector3 vtemp2 = v3 - v2;
        angle = abs(vtemp1.angle(vtemp2));
        if(angle > 0.35)
        {
            return 100000.0 * abs(tan(angle));
        } else {
            return abs(tan(angle)) * 10.0;
        }
    }

    double outCirclePenalty(tf::Vector3 v1, tf::Vector3 v2, tf::Vector3 v3){

    }

    double distance2Point(double x1, double y1, double x2, double y2){
        return pow(x2 - x1, 2.0) + pow(y2 - y1, 2.0);
    }

    double sign(double x){
        if(x < 0.0)
        {
            return -1.0;
        }else {
            return 1.0;
        }
    }

    double signedAngle(tf::Vector3 v1, tf::Vector3 v2){

        return atan2( v1.x() * v2.y() - v1.y() * v2.x(), v1.x() * v2.x() + v1.y() * v2.y());
    }

    double calRad(double len, double angleDelta)
    {
        return len / (2.0 * sin(angleDelta / 2.0));
    }

    double TwoPointDistance(tf::Vector3 x1, tf::Vector3 x2)
    {
        tf::Vector3 dv = {x2.x() - x1.x(), x2.y() - x1.y(), 0.0};
        tf::Vector3 sa = {cos(x1.z()), sin(x1.z()), 0.0};
        tf::Vector3 ea = {cos(x2.z()), sin(x2.z()), 0.0};
        double ang1 = signedAngle(dv.normalized(), sa);
        double ang2 = signedAngle(dv.normalized(), ea);
        double d1 = 0.0;
        if(abs(ang1) + abs(ang2) > M_PI)
        {
            d1 = calRad(dv.length(), (2 * M_PI - (abs(ang1) + abs(ang2)))) * (2 * M_PI - (abs(ang1) + abs(ang2)));
        } else {
            d1 = calRad(dv.length(), abs(ang1) + abs(ang2)) * (abs(ang1) + abs(ang2));
        }
        if(isnan(d1)) 
            d1 = dv.length();
        return d1;

    }
    //add data is few initial points and number of 
    double targetFunction(const std::vector<double> &x, std::vector<double> &grad, void *data)
    {
        dataInitial *d = reinterpret_cast<dataInitial*>(data);
        /*if (!grad.empty()) {
            for(int i = 1; i < x.size() / 2 - 1; i++)
            {
                grad[i * 2] = 4 * x[i * 2] - 2 * x[(i - 1) * 2] - 2 * x[(i + 1) * 2];
                grad[i * 2 + 1] = 4 * x[i * 2 + 1] - 2 * x[(i - 1) * 2 + 1] - 2 * x[(i + 1) * 2 + 1]; 
            }
            grad[0] = 4 * x[0] - 2 * d->start.x() - 2 * x[2];
            grad[1] = 4 * x[1] - 2 * d->start.y() - 2 * x[3];
            grad[x.size() - 2] = 4 * x[x.size() - 2] - 2 * d->end.x() - 2 * x[x.size() - 4];
            grad[x.size() - 1] = 4 * x[x.size() - 1] - 2 * d->end.y() - 2 * x[x.size() - 3];
        }*/
        /*
        double val = 0.0;
        double angle = 0.0;
        val += distance2Point(d->start.x(), d->start.y(), x[0], x[1]);      
        for(int i = 1; i < x.size() / 3; i++)
        {
            val += distance2Point(x[(i - 1) * 3], x[(i - 1) * 3 + 1], x[i * 3], x[i * 3 + 1]);
        }
        int totalNum = x.size() - 1;
        val += distance2Point(x[totalNum - 2], x[totalNum - 1], d->end.x(), d->end.y()); 
        
       */
      double val = 0.0;
       double di = TwoPointDistance(d->start, {x[0], x[1], x[2]});
       int xs = x.size();
        val +=  abs(di) * abs(di);
        for(int i = 1; i < x.size() / 3; i++)
        {
            di = TwoPointDistance({x[(i - 1) * 3], x[(i - 1) * 3 + 1], x[(i - 1) * 3 + 2]}, {x[i * 3], x[i * 3 + 1], x[i * 3 + 2]});
            val += abs(di) * abs(di);
        }
        di = TwoPointDistance({x[xs - 3], x[xs - 2], x[xs - 1]}, d->end);
        val +=  abs(di) * abs(di);
        
       return val;
    }



    double ThreePointReachable(tf::Vector3 x1, tf::Vector3 x2, tf::Vector3 x3)
    {
        tf::Vector3 dv = {x2.x() - x1.x(), x2.y() - x1.y(), 0.0};
        tf::Vector3 sa = {cos(x1.z()), sin(x1.z()), 0.0};
        tf::Vector3 ea = {cos(x2.z()), sin(x2.z()), 0.0};
        double ang1 = signedAngle(dv.normalized(), sa);
        double ang2 = signedAngle(dv.normalized(), ea);
        double bck = abs(ang2 + ang1);
        double r1 = calRad(dv.length(), abs(ang1) + abs(ang2));
        dv = {x3.x() - x2.x(), x3.y() - x2.y(), 0.0};
        sa = {cos(x2.z()), sin(x2.z()), 0.0};
        ea = {cos(x3.z()), sin(x3.z()), 0.0};
        ang1 = signedAngle(dv.normalized(), sa);
        ang2 = signedAngle(dv.normalized(), ea);
        double frw = abs(ang2 + ang1);
        double r2 = calRad(dv.length(), abs(ang1) + abs(ang2));
        if(isnan(r1)) 
            r1 = 1000.0;
        if(isnan(r2))
            r2 = 1000.0;
        if(bck > 0.01)
        {
            return bck;
        }
        if(frw > 0.01)
        {
            return frw;
        }
        if(r1 < 2.6)
        {
            return 2.6 - r1;
        }
        if(r2 < 2.6)
        {
            return 2.6 - r2;
        }
        return 5.2 - r1 - r2;

    }


    double TwoPointReachable(tf::Vector3 x1, tf::Vector3 x2)
    {
        tf::Vector3 dv = {x2.x() - x1.x(), x2.y() - x1.y(), 0.0};
        tf::Vector3 sa = {cos(x1.z()), sin(x1.z()), 0.0};
        tf::Vector3 ea = {cos(x2.z()), sin(x2.z()), 0.0};
        double ang1 = signedAngle(dv.normalized(), sa);
        double ang2 = signedAngle(dv.normalized(), ea);
        if(abs(ang1) > M_PI_2 && abs(ang2) > M_PI_2)
        {
            ang1 = (M_PI - abs(ang1)) * sign(ang1);
            ang2 = (M_PI - abs(ang2)) * sign(ang2);
        }
        double bck = abs(ang2 + ang1);
        return bck - 0.01;

    }

    double TwoPointRadConst(tf::Vector3 x1, tf::Vector3 x2)
    {
        tf::Vector3 dv = {x2.x() - x1.x(), x2.y() - x1.y(), 0.0};
        tf::Vector3 sa = {cos(x1.z()), sin(x1.z()), 0.0};
        tf::Vector3 ea = {cos(x2.z()), sin(x2.z()), 0.0};
        double ang1 = signedAngle(dv.normalized(), sa);
        double ang2 = signedAngle(dv.normalized(), ea);
        double r1 = 0.0;
        if(abs(ang1) + abs(ang2) > M_PI)
        {
            r1 = calRad(dv.length(), 2 * M_PI - (abs(ang1) + abs(ang2)));
        } else {
            r1 = calRad(dv.length(), abs(ang1) + abs(ang2));
        }
        
        if(isnan(r1)) 
            r1 = 1000.0;
        
        return 2.6 - r1;

    }


    double angleConstraints(const std::vector<double> &x, std::vector<double> &grad, void *data)
    {
        dataConstraints *d = reinterpret_cast<dataConstraints*>(data);
        int i = d->number;
        int maxNumber = d->maxNumber;
        if(i == 0)
        {
            return ThreePointReachable(d->start, {x[i * 3], x[i * 3 + 1], x[i * 3 + 2]}, {x[(i + 1) * 3], x[(i + 1) * 3 + 1], x[(i + 1) * 3 + 2]});
        }
        if(i == d->maxNumber)
        {
            return ThreePointReachable({x[(i - 1) * 3], x[(i - 1) * 3 + 1], x[(i - 1) * 3 + 2]}, {x[i * 3], x[i * 3 + 1], x[i * 3 + 2]}, d->end);
        }
        return ThreePointReachable({x[(i - 1) * 3], x[(i - 1) * 3 + 1], x[(i - 1) * 3 + 2]}, {x[i * 3], x[i * 3 + 1], x[i * 3 + 2]}, {x[(i + 1) * 3], x[(i + 1) * 3 + 1], x[(i + 1) * 3 + 2]});
    }

    double bckTwoPointA(const std::vector<double> &x, std::vector<double> &grad, void *data)
    {
        dataConstraints *d = reinterpret_cast<dataConstraints*>(data);
        int i = d->number;
        int maxNumber = d->maxNumber;
        if(i == 0)
        {
            return TwoPointReachable(d->start, {x[0], x[1], x[2]});
        }
        if(i != 0)
        {
            return TwoPointReachable({x[(i - 1) * 3], x[(i - 1) * 3 + 1], x[(i - 1) * 3 + 2]}, {x[i * 3], x[i * 3 + 1], x[i * 3 + 2]});
        }
    }

    double bckTwoPointR(const std::vector<double> &x, std::vector<double> &grad, void *data)
    {
        dataConstraints *d = reinterpret_cast<dataConstraints*>(data);
        int i = d->number;
        int maxNumber = d->maxNumber;
        if(i == 0)
        {
            return TwoPointRadConst(d->start, {x[0], x[1], x[2]});
        }
        if(i != 0)
        {
            return TwoPointRadConst({x[(i - 1) * 3], x[(i - 1) * 3 + 1], x[(i - 1) * 3 + 2]}, {x[i * 3], x[i * 3 + 1], x[i * 3 + 2]});
        }
    }

    double frwTwoPointA(const std::vector<double> &x, std::vector<double> &grad, void *data)
    {
        dataConstraints *d = reinterpret_cast<dataConstraints*>(data);
        int i = d->number;
        if(i != d->maxNumber)
        {
            return TwoPointReachable({x[i * 3], x[i * 3 + 1], x[i * 3 + 2]}, {x[(i + 1) * 3], x[(i + 1) * 3 + 1], x[(i + 1) * 3 + 2]});
        }
        if(i == d->maxNumber)
        {
            return TwoPointReachable({x[i * 3], x[i * 3 + 1], x[i * 3 + 2]}, d->end);
        }
    }

    double frwTwoPointR(const std::vector<double> &x, std::vector<double> &grad, void *data)
    {
        dataConstraints *d = reinterpret_cast<dataConstraints*>(data);
        int i = d->number;
        if(i != d->maxNumber)
        {
            return TwoPointRadConst({x[i * 3], x[i * 3 + 1], x[i * 3 + 2]}, {x[(i + 1) * 3], x[(i + 1) * 3 + 1], x[(i + 1) * 3 + 2]});
        }
        if(i == d->maxNumber)
        {
            return TwoPointRadConst({x[i * 3], x[i * 3 + 1], x[i * 3 + 2]}, d->end);
        }
    }

    double constraints(const std::vector<double> &x, std::vector<double> &grad, void *data)
    {
        dataConstraints *d = reinterpret_cast<dataConstraints*>(data);
        /*if (!grad.empty()) {
            for(int i = 0; i < x.size() / 2; i++)
            {
                if(i == d->number)
                {
                    grad[2 * i] = x[i * 2] * 2 - 2;
                    grad[2 * i + 1] = x[i * 2 + 1] * 2 - 2;
                }else{
                    grad[2 * i] = 0;
                    grad[2 * i + 1] = 0;
                }
            }
        }*/
        double val1 = x[d->number * 3] - d->bubble->pose.x();
        double val2 = x[d->number * 3 + 1] - d->bubble->pose.y();
        double r = d->bubble->radius;
        return val1 * val1 + val2 * val2 - r * r;
    }

    double angleNorm(double angle){
        angle = fmod(angle, M_PI * 2);
        if(angle < 0.0)
        {
            angle = M_PI * 2 + angle;
        }
        return angle;
    }

    void GlobalPlanner::convexOptimization(std::vector<Bubble *> &listOfBubbles, std::vector<double> &optPos) {
        std::vector<Bubble *> listBubbles;
        for(int i = 0; i < listOfBubbles.size(); i++)
        {
            if(i != 0 && i != listOfBubbles.size() - 1)
            {
                listBubbles.push_back(listOfBubbles[i]);
            }
        }
        nlopt::opt opt(nlopt::LN_COBYLA , listBubbles.size() * 3);
        

        dataInitial initData;
        initData.start = {listOfBubbles[0]->pose.x(), listOfBubbles[0]->pose.y(), angleNorm(this->startYaw)};
        initData.end = {listOfBubbles[listOfBubbles.size() - 1]->pose.x(), listOfBubbles[listOfBubbles.size() - 1]->pose.y(), angleNorm(listOfBubbles[listOfBubbles.size() - 1]->theta)};
        opt.set_min_objective(targetFunction, &initData);
        dataConstraints data[listBubbles.size()];
        int ii = 1151;
        for(int i = 0; i < listBubbles.size(); i++)
        {
            data[i].number = i;
            data[i].bubble = listBubbles[i];
            data[i].maxNumber = listBubbles.size() - 1;
            ROS_ERROR("num of bubbles %lu", listBubbles.size());
        }
        data[0].start = {listOfBubbles[0]->pose.x(), listOfBubbles[0]->pose.y(), angleNorm(this->startYaw)};
        data[listBubbles.size() - 1].end = {listOfBubbles[listOfBubbles.size() - 1]->pose.x(), listOfBubbles[listOfBubbles.size() - 1]->pose.y(), angleNorm(listOfBubbles[listOfBubbles.size() - 1]->theta)};
        
        for(dataConstraints & item : data)
        {
            opt.add_inequality_constraint(constraints, &item, 1e-2);
            opt.add_inequality_constraint(bckTwoPointA, &item, 1e-2);
            opt.add_inequality_constraint(bckTwoPointR, &item, 1e-2);
            opt.add_inequality_constraint(frwTwoPointA, &item, 1e-2);
            opt.add_inequality_constraint(frwTwoPointR, &item, 1e-2);
            
        }
        ROS_ERROR("error 9");
        opt.set_xtol_rel(1e-1);
        std::vector<double> lb;
        std::vector<double> ub;
        std::vector<double> x(listBubbles.size() * 3);
        for(int i = 0; i < listBubbles.size(); i++)
        {
            x[i * 3] = listBubbles[i]->pose.x();
            x[i * 3 + 1] = listBubbles[i]->pose.y();
            x[i * 3 + 2] = angleNorm(listBubbles[i]->theta);
            lb.push_back(listBubbles[i]->pose.x() - listBubbles[i]->radius);
            lb.push_back(listBubbles[i]->pose.y() - listBubbles[i]->radius);
            lb.push_back(0.0);
            ub.push_back(listBubbles[i]->pose.x() + listBubbles[i]->radius);
            ub.push_back(listBubbles[i]->pose.y() + listBubbles[i]->radius);
            ub.push_back(2 * M_PI);
            ii++;
        }
        ROS_ERROR("num of x-es %lu", x.size());
        opt.set_lower_bounds(lb);
        opt.set_upper_bounds(ub);
        ROS_ERROR("error 10");
        double minf;

        try{
            ROS_ERROR("error 11");
            nlopt::result result = opt.optimize(x, minf);
            ROS_ERROR("error 12");
        }
        catch(std::exception &e) {
            std::cout << "nlopt failed: " << e.what() << std::endl;
        }
        ROS_ERROR("error 13");
        for(int i = 0; i < listBubbles.size(); i++)
        {
            //this->ma.markers.push_back(this->createVizCylinder({listBubbles[i]->pose.x(), listBubbles[i]->pose.y()}, listBubbles[i]->radius, i));
        }
        float pathLength = 0.0;
        /*
        for(int i = 1; i < listBubbles.size(); i++)
        {
            this->ma.markers.push_back(this->createArrow({x[i * 3], x[i * 3 + 1], x[i * 3 + 2]}, i + 634634));
            this->ma.markers.push_back(this->createLine({x[(i - 1) * 3], x[(i - 1) * 3 + 1]}, {x[i * 3], x[i * 3 + 1]}, i + 34244));
            pathLength += tf::tfDistance({x[(i - 1) * 3], x[(i - 1) * 3 + 1], 0.0}, {x[i * 3], x[i * 3 + 1], 0.0});
            tf::Vector3 pointVector = {x[i * 3] - x[(i - 1) * 3], x[i * 3 + 1] - x[(i - 1) * 3 + 1], 0.0};
            tf::Vector3 startAngVec = {cos(x[(i - 1) * 3 + 2]), sin(x[(i - 1) * 3 + 2]), 0.0};
            tf::Vector3 endAngVec = {cos(x[i * 3 + 2]), sin(x[i * 3 + 2]), 0.0};
            double ang1 = signedAngle(pointVector.normalized(), startAngVec );
            double ang2 = signedAngle(pointVector.normalized(), endAngVec);
        }
        */
        for(int i = 0; i < listBubbles.size(); i++)
        {
            optPos.push_back(x[i * 3]);
            optPos.push_back(x[i * 3 + 1]);
            optPos.push_back(x[i * 3 + 2]);
        }
        /*
        this->ma.markers.push_back(this->createArrow({initData.start.x(), initData.start.y(), initData.start.z()}, 24242));
        this->ma.markers.push_back(this->createArrow({initData.end.x(), initData.end.y(), initData.end.z()}, 124324));
        this->ma.markers.push_back(this->createLine({x[0], x[1]}, {initData.start.x(), initData.start.y()}, 42144));
        this->ma.markers.push_back(this->createLine({x[x.size() - 3], x[x.size() - 2]}, {initData.end.x(), initData.end.y()}, 4244));
        */
        pathLength += tf::tfDistance({x[0], x[1], 0.0}, {initData.start.x(), initData.start.y(), 0.0});
        pathLength += tf::tfDistance({x[x.size() - 3], x[x.size() - 2], 0.0}, {initData.end.x(), initData.end.y(), 0.0});
        ROS_ERROR("length of optimized path - %f", pathLength);
        this->publisher.publish(this->ma);       
        ROS_ERROR("error 14");
    }


    visualization_msgs::Marker GlobalPlanner::createArrow(tf::Vector3 start, int id)
    {
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = "/map";
        arrow.header.stamp = ros::Time::now();
        arrow.ns = "arrow";
        arrow.action = visualization_msgs::Marker::ADD;
        tf::Quaternion myQuaternion;
        myQuaternion.setRPY(0, 0, start.z());
        tf::quaternionTFToMsg(myQuaternion, arrow.pose.orientation);
        arrow.id = id;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.scale.y = 0.05;
        arrow.scale.z = 0.05;
        arrow.scale.x = 0.30;
        arrow.color.b = 1.0;
        arrow.color.a = 1.0;

        arrow.pose.position.x = start.x();
        arrow.pose.position.y = start.y();
        arrow.pose.position.z = 0.2;
        return arrow;
    }

    visualization_msgs::Marker GlobalPlanner::createLine(std::pair<float, float> start, std::pair<float, float> end, int id)
    {
        visualization_msgs::Marker line;
        line.header.frame_id = "map";
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
        lineStart.z = 0.0;
        lineEnd.x = end.first;
        lineEnd.y = end.second;
        lineEnd.z = 0.0;
        line.points.push_back(lineStart);
        line.points.push_back(lineEnd);
        return line;
    }

    visualization_msgs::Marker GlobalPlanner::createVizPoint(std::pair<float, float> pos, int id)
    {
        visualization_msgs::Marker point;
        point.header.frame_id = "map";
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
        p.position.z = 0.2;
        p.orientation.w = 1.0;
        point.pose = p;
        return point;
    }

    visualization_msgs::Marker GlobalPlanner::createVizCylinder(std::pair<float, float> pos, float radius, int id)
    {
        visualization_msgs::Marker point;
        point.header.frame_id = "map";
        point.header.stamp = ros::Time::now();
        point.ns = "cylinders";
        point.action = visualization_msgs::Marker::ADD;
        point.pose.orientation.w = 1.0;
        point.id = id;
        point.type = visualization_msgs::Marker::CYLINDER;
        point.scale.x = radius * 2.0;
        point.scale.y = radius * 2.0;
        point.scale.z = 0.1;
        point.color.g = 1.0f;
        point.color.a = 0.2;
        geometry_msgs::Pose p;
        p.position.x = pos.first ;
        p.position.y = pos.second ;
        p.position.z = 0.2;
        p.orientation.w = 1.0;
        point.pose = p;
        return point;
    }

    GlobalPlanner::GlobalPlanner()
    {
        ROS_INFO("starting to create path!!!!@!@!");
    }

    GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        ROS_INFO("starting to create path!!!!@!@!");
        initialize(name, costmap_ros);
    }

    void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        ROS_INFO("starting to create path!!!!@!@!");
        this->map = costmap_ros;
        this->nh = ros::NodeHandle("~");
        this->publisher = this->nh.advertise<visualization_msgs::MarkerArray>("/vizmrks", 100);
    }

    //add bias or zoom heuristic
    tf::Vector3 GlobalPlanner::genSample(const std::vector<tf::Vector3> &constraints, int id = 0)
    {
        float coin = (static_cast<float>(rand()) / static_cast<float>(RAND_MAX));
        if (coin < 0.05)
        {
            return tf::Vector3(constraints[1]);
        }
        tf::Vector3 newSample = {1000.0, 1000.0, 1000.0};
        float dist = tf::tfDistance(constraints[0], constraints[1]);
        tf::Vector3 center = (constraints[1] - constraints[0]) / 2.0 + constraints[0];
        newSample.setZ(0);
        newSample.setX(center.x() + (-dist + (2 * dist) * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX))));
        newSample.setY(center.y() + (-dist + (2 * dist) * (static_cast<float>(rand()) / static_cast<float>(RAND_MAX))));
        return newSample;
    }


    State *GlobalPlanner::findNearestNode(kd_tree::KDTree<State *> &tree, const tf::Vector3 sample)
    {
        kd_tree::KDNode<rrt_planner::State *> *_closestNode = NULL;
        tree.NNSearch({sample.x(), sample.y()}, _closestNode);
        return _closestNode->data;
    }

    bool GlobalPlanner::collisionCheck(tf::Vector3 pos, unsigned char targetCost)
    {
        uint x = 0;
        uint y = 0;
        bool isInsideMap = this->map->getCostmap()->worldToMap(pos.x(), pos.y(), x, y);
        if (isInsideMap != true)
        {
            return false;
        }
        unsigned char cost = this->map->getCostmap()->getCost(x, y);
        if (cost > targetCost)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    tf::Vector3 GlobalPlanner::getKinematicsDerivatives(State *pose, float steeringAngle, float vel)
    {
        tf::Vector3 state3d;
        state3d.setX(vel * cos(pose->theta));
        state3d.setY(vel * sin(pose->theta));
        state3d.setZ((vel / this->carLength) * tan(steeringAngle));
        return state3d;
    }

    State *GlobalPlanner::solveRK(State *pose, float steeringAngle, float vel)
    {
        tf::Vector3 k1 = this->getKinematicsDerivatives(pose, steeringAngle, vel);
        tf::Vector3 k2 = this->getKinematicsDerivatives(pose + State(tf::Vector3(k1.x() / 2, k1.y() / 2, 0.0), k1.z() / 2), steeringAngle, vel);
        tf::Vector3 k3 = this->getKinematicsDerivatives(pose + State(tf::Vector3(k2.x() / 2, k2.y() / 2, 0.0), k2.z() / 2), steeringAngle, vel);
        tf::Vector3 k4 = this->getKinematicsDerivatives(pose + State(tf::Vector3(k3.x(), k3.y(), 0.0), k3.z()), steeringAngle, vel);
        tf::Vector3 accK = (1.0 / 6.0) * this->deltaTime * (k1 + 2 * k2 + 2 * k3 + k4);
        return new State(tf::Vector3(pose->pose.x() + accK.x(), pose->pose.y() + accK.y(), 0), pose->theta + accK.z());
    }

    State *GlobalPlanner::genState(State *closestNode, tf::Vector3 sample)
    {
        State *_bestState = NULL;
        float _minDist = std::numeric_limits<float>::infinity(); // currently for closest x, y, but rework for x, y, theta
        float bestAngle = -101;                                  //for all in obstacles check

        for (float i = -this->maxSteeringAngle; i < this->maxSteeringAngle; i += 0.05)
        {
            State *tempState = this->solveRK(closestNode, i);
            tempState->parent = closestNode;
            if (this->collisionCheck(tempState->pose))
            {
                float _currDist = tf::tfDistance(sample, tempState->pose);
                if (_currDist < _minDist)
                {
                    _minDist = _currDist;
                    bestAngle = i;
                    _bestState = tempState;
                }
            }
        }
        return _bestState;
    }

    bool GlobalPlanner::bubbleCollisionCheck(Bubble *point, tf::Vector3 &obstacleDir, float radius)
    {
        uint x = 0;
        uint y = 0;
        tf::Vector3 cPoint;        
        for (float ang = 0; ang < 2 * M_PI; ang += 2.0 * M_PI / 360.0)
        {
            cPoint.setX(point->pose.x() + radius * cos(ang));
            cPoint.setY(point->pose.y() + radius * sin(ang));
            cPoint.setZ(this->zpose);

            bool isInsideMap = this->map->getCostmap()->worldToMap(cPoint.x(), cPoint.y(), x, y);
            if (isInsideMap != true)
            {
                continue;
            }
            unsigned char cost = this->map->getCostmap()->getCost(x, y);
            if (cost > 130)
            {
                obstacleDir = cPoint; 
                return false;
            }
        }
        return true;
    }

    Bubble* GlobalPlanner::translateBubble(State* point)
    {
        tf::Vector3 obstacleDir;
        Bubble * bubble = new Bubble();
        bubble->pose = point->pose;
        bubble->radius = this->lowerBound;
        bubble->theta = point->theta;
        float step = this->lowerBound / 2.0;
        float eps = 0.05;
        while(abs(step) > eps )
        {
            
            if(this->bubbleCollisionCheck(bubble, obstacleDir, this->lowerBound))
            {
                return bubble;
            } 
            bubble->pose -= step * obstacleDir.normalized();
            step /= 2.0;
        }
        return bubble;
    }

    Bubble *GlobalPlanner::generateBubble(State *point, float lastBubbleRadius)
    {
        tf::Vector3 obstacleDir;
        Bubble * bubble = new Bubble();
        bubble->theta = point->theta;
        bubble->pose = point->pose;
        lastBubbleRadius = this->lowerBound;
        float eps = 0.1;
        if (this->bubbleCollisionCheck(bubble, obstacleDir, this->lowerBound))
        {
            float step = 0.125; 
            float lastOkRadius = 0.0;
            while (lastBubbleRadius < this->upperBound + 0.1 && lastBubbleRadius > this->lowerBound - 0.01)
            {
                if (this->bubbleCollisionCheck(bubble, obstacleDir, lastBubbleRadius))
                {
                    lastOkRadius = lastBubbleRadius;
                    lastBubbleRadius += step;
                }
                else
                {
                    break;
                }
            }
            if(lastOkRadius < 0.01)
            {
                bubble->radius = lastOkRadius;
                return bubble;
                return this->translateBubble(point);
            }else{
                bubble->radius = lastOkRadius;
                return bubble;
            }
        }
        else
        {
            bubble->radius = 0.25;
            return bubble;
            return this->translateBubble(point);
        }
    }

    void GlobalPlanner::bubbleTubeCreator(std::vector<State *> &path, std::vector<Bubble *> &tube)
    {
        float lastBubbleRadius = 0.0;
        tube.push_back(new Bubble(path[0]->pose, 0.0, path[0]->theta));
        int cursor = 0;
        for (int i = 1; i < path.size() - 1; i++)
        {
            if (tf::tfDistance(path[i]->pose, tube[cursor]->pose) > 0.5 * tube[cursor]->radius)
            {
                tube.push_back(this->generateBubble(path[i], lastBubbleRadius));
                cursor++;
            }
        }
        tube.push_back(new Bubble(path[path.size() - 1]->pose, 0.0, path[path.size() - 1]->theta));
        int i = 20000;
    }

    bool GlobalPlanner::checkReedShepp(State * state, const geometry_msgs::PoseStamped goal) {
        ompl::base::ReedsSheppStateSpace* RSspace = new ompl::base::ReedsSheppStateSpace(2.7);
        ompl::base::StateSpacePtr space(RSspace);
        ompl::base::ScopedState<ompl::base::SE2StateSpace> RSstart(space);
        RSstart->setXY(state->pose.x(), state->pose.y());
        RSstart->setYaw(state->theta);
        ompl::base::ScopedState<ompl::base::SE2StateSpace> RSgoal(space);
        RSgoal->setXY(goal.pose.position.x, goal.pose.position.y);
        RSgoal->setYaw(this->goalYaw);
        ompl::base::ScopedState<ompl::base::SE2StateSpace> tempState(space);
        for(int i = 0; i < 20; i++)
        {
            RSspace->interpolate(RSstart.get(), RSgoal.get(), 0.05 * i, tempState.get());
            if(this->collisionCheck({tempState->getX(), tempState->getY(), 0.0}, 0) == false)
            {
                return false;
            }
        }
        return true;
    }

    std::vector<geometry_msgs::PoseStamped> GlobalPlanner::createRRTPath(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal)
    {
        // first val - left bottom corener of ranfom field second val is top right corener of rand field
        /*std::vector<tf::Vector3> generationConstraights = {{start.pose.position.x, start.pose.position.y, this->zpose}, {goal.pose.position.x, goal.pose.position.y, this->zpose}};
        kd_tree::KDTree<rrt_planner::State *> tree;
        tf::Vector3 initPose = tf::Vector3(start.ppos.x, start.ppos.y, 0);
        tf::Quaternion initRotation = tf::Quaternion(start.pq.x, start.pq.y, start.pq.z, start.pq.w);
        double roll, pitch, yaw;

        tf::Matrix3x3(initRotation).getRPY(roll, pitch, yaw);

        tree.insert({initPose.x(), initPose.y()}, new State(initPose, yaw, NULL, 0.0));
        auto begin = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < 8000; i++)
        {
            tf::Vector3 sample = this->genSample(generationConstraights, i);
            State *NNNode = this->findNearestNode(tree, sample);
            State *newSample = this->genState(NNNode, sample);
            if (newSample != NULL)
            {
                newSample->cost = NNNode->cost + tf::tfDistance(NNNode->pose, newSample->pose);
                tree.insert({newSample->pose.x(), newSample->pose.y()}, newSample);
            }
        }
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> elapsed = finish - begin;
        ROS_ERROR("comp time123: %f", elapsed.count());
        kd_tree::KDNode<rrt_planner::State *> *closestNodeToGoalKDNode = NULL;
        tree.NNSearch({goal.ppos.x, goal.ppos.y}, closestNodeToGoalKDNode);
        std::vector<kd_tree::KDNode<rrt_planner::State *> *> closestToGoal;
        tree.RangeSearch({goal.ppos.x - 1.75, goal.ppos.x + 1.75}, {goal.ppos.y - 1.75, goal.ppos.y + 1.75}, closestToGoal);
        float smallesCost = 100000.0;
        State *closestNodeToGoal;
        for(kd_tree::KDNode<rrt_planner::State *>* item : closestToGoal)
        {
            if(item->data->cost < smallesCost)
            {
                smallesCost = item->data->cost;
                closestNodeToGoal = item->data;
            }
        }
        std::vector<State *> rrtPath;
        rrtPath.push_back(new State({goal.ppos.x, goal.ppos.y, this->zpose}, 0.0));
        while (closestNodeToGoal != NULL)
        {
            rrtPath.push_back(closestNodeToGoal);
            closestNodeToGoal = closestNodeToGoal->parent;
        }*/
        //Hybrid

        
        tf::Quaternion initRotation = tf::Quaternion(start.pq.x, start.pq.y, start.pq.z, start.pq.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(initRotation).getRPY(roll, pitch, yaw);
        tf::Vector3 initPose = tf::Vector3(start.ppos.x, start.ppos.y, yaw);
        tf::Quaternion goalRotation = tf::Quaternion(goal.pq.x, goal.pq.y, goal.pq.z, goal.pq.w);
        roll, pitch, yaw;
        tf::Matrix3x3(goalRotation).getRPY(roll, pitch, yaw);
        tf::Vector3 goalPose = tf::Vector3(goal.ppos.x, goal.ppos.y, this->goalYaw);
        AStarInterface algo = AStarInterface(this->map->getCostmap(), goalPose, 2);
        continuousState* head;
        algo.opened.push(new continuousState(initPose.x(), initPose.y(), initPose.z()));
        int k = 0; //текущая итерация
        int kk = 15; //вызов Reed-Shepp
        auto startt = chrono::high_resolution_clock::now(); 
        while(!algo.opened.empty())
        {
            
            continuousState* bestCostState = algo.popOpen();
            k++;
            float dToGoal = sqrt((bestCostState->x - goalPose.x())*(bestCostState->x - goalPose.x()) +  (bestCostState->y - goalPose.y())*(bestCostState->y - goalPose.y()));
            if(dToGoal < 0.5)
            {
                head = bestCostState;
                break;
            }
            if(dToGoal < kk)
            {
                kk = (int)dToGoal + 1;
            }
            discreteState* temp = algo.getClosed(bestCostState);
            if(temp->value == 2)
            {
                continue;
            }
            else
            {
                temp->value = 2;
                if(k % kk == 0)
                {
                    ompl::base::ReedsSheppStateSpace* RSspace = new ompl::base::ReedsSheppStateSpace(2.6);
                    ompl::base::StateSpacePtr space(RSspace);
                    ompl::base::ScopedState<ompl::base::SE2StateSpace> RSstart(space);
                    RSstart->setXY(bestCostState->x, bestCostState->y);
                    RSstart->setYaw(bestCostState->theta);
                    ompl::base::ScopedState<ompl::base::SE2StateSpace> RSgoal(space);
                    RSgoal->setXY(goalPose.x(), goalPose.y());
                    RSgoal->setYaw(goalPose.z());
                    ompl::base::ScopedState<ompl::base::SE2StateSpace> tempState(space);
                    bool progress = true;
                    for(int i = 1; i < 20; i++)
                    {
                        RSspace->interpolate(RSstart.get(), RSgoal.get(), 0.05 * i, tempState.get());
                        progress = progress && this->collisionCheck({tempState->getX(), tempState->getY(), tempState->getYaw()}, 0);
                    }

                    if(progress == true)
                    {
                        RSspace->interpolate(RSstart.get(), RSgoal.get(), 0.05, tempState.get());
                        continuousState* RSpath = new continuousState(tempState->getX(), tempState->getY(), tempState->getYaw(), bestCostState);
                        for(int i = 2; i < 20; i++)
                        {
                            RSspace->interpolate(RSstart.get(), RSgoal.get(), 0.05 * i, tempState.get());
                            RSpath = new continuousState(tempState->getX(), tempState->getY(), tempState->getYaw(), RSpath);
                        }
                        ROS_ERROR("Length of non optimized - %f", algo.countLength(bestCostState));
                        head = RSpath;
                        std::vector<Bubble *> bubbleArray;
                        std::vector<State *> statePath = algo.buildStatePath(bestCostState);
                        if(statePath.size() < 2)
                        {
                            break;
                        }
                        this->bubbleTubeCreator(statePath, bubbleArray);
                        int kk = 124;
                        int z = 0;
                        for(Bubble*& b : bubbleArray)
                        {
                            //this->ma.markers.push_back(this->createVizCylinder({b->pose.x(), b->pose.y()}, b->radius, kk));
                            //this->ma.markers.push_back(this->createArrow({b->pose.x(), b->pose.y(), b->theta}, kk + kk + kk));
                            kk++;
                            z++;
                        }
                        auto startt_o = chrono::high_resolution_clock::now();
                        if(bubbleArray.size() < 4)
                        {
                            break;
                        }
                        std::vector<double> optPath;
                        this->convexOptimization(bubbleArray, optPath);
                        continuousState* optTemp = new continuousState(optPath[0], optPath[1], optPath[2], nullptr);
                        for(int i = 1; i < optPath.size() / 3; i++)
                        {
                            ROS_ERROR("optPos - %f %f", optPath[i * 3], optPath[i * 3 + 1]);
                            optTemp = new continuousState(optPath[i * 3], optPath[i * 3 + 1], optPath[i * 3 + 2], optTemp);
                        }
                        continuousState * tempBestCost = bestCostState;
                        int kki = 0;
                        while(tempBestCost->parent != nullptr)
                        {
                            this->ma.markers.push_back(this->createLine({tempBestCost->parent->x , tempBestCost->parent->y}, {tempBestCost->x , tempBestCost->y}, 424 + kki));
                            kki++;
                            tempBestCost = tempBestCost->parent;
                        }
                        bestCostState->parent = optTemp;
                        auto endt_o = chrono::high_resolution_clock::now(); 
                        double time_taken_o = chrono::duration_cast<chrono::nanoseconds>(endt_o - startt_o).count();
                        time_taken_o *= 1e-9;
                        ROS_ERROR("optimization - %f", time_taken_o);
                        break;
                    }
                }
                for (float i = -this->maxSteeringAngle; i < this->maxSteeringAngle; i += 0.1)
                {
                    State* stateHolder = this->solveRK(new State({bestCostState->x, bestCostState->y, 0.0}, bestCostState->theta), i, 0.5);
                    stateHolder->theta = fmod(stateHolder->theta, M_PI * 2);
                    if(stateHolder->theta < 0.0)
                    {
                        stateHolder->theta = M_PI * 2 + stateHolder->theta;
                    }
                    if (this->collisionCheck(stateHolder->pose, 0))
                    {
                        
                        continuousState* castState = algo.stateCast(stateHolder, bestCostState);
                        discreteState* temp = algo.getClosed(castState);
                        if(temp->value != 2)
                        {
                            algo.pushOpen(castState);
                        }
                    }
                }
                for (float i = -this->maxSteeringAngle; i < this->maxSteeringAngle; i += 0.07)
                {
                    State* stateHolder = this->solveRK(new State({bestCostState->x, bestCostState->y, 0.0}, bestCostState->theta), i, -0.5);
                    stateHolder->theta = fmod(stateHolder->theta, M_PI * 2);
                    if(stateHolder->theta < 0.0)
                    {
                        stateHolder->theta = M_PI * 2 + stateHolder->theta;
                    }
                    if (this->collisionCheck(stateHolder->pose, 0))
                    {
                        
                        continuousState* castState = algo.stateCast(stateHolder, bestCostState);
                        discreteState* temp = algo.getClosed(castState);
                        if(temp->value != 2)
                        {
                            algo.pushOpen(castState, 2.0);
                        }
                    }
                }
            }

        }
        auto endt = chrono::high_resolution_clock::now(); 
        double time_taken = chrono::duration_cast<chrono::nanoseconds>(endt - startt).count();
        time_taken *= 1e-9;
        ROS_ERROR("%d", k);
        ROS_ERROR("total time - %f", time_taken);
        return algo.buildPath(head, this->header);
        /*


        std::reverse(std::begin(rrtPath), std::end(rrtPath));
        for(int i = 0; i < rrtPath.size(); i+=5)
        {
            if(this->checkReedShepp(rrtPath[i], goal)){
                rrtPath.resize(i + 1);
                break;
            }
        }
        if(rrtPath.size() > 2)
        {
        std::vector<Bubble *> bubbleArray;
        ROS_ERROR("bubble tube creator");
        this->bubbleTubeCreator(rrtPath, bubbleArray);
        ROS_ERROR("bubble tube creator end");
        try
        {
            this->convexOptimization(bubbleArray);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        }
        ompl::base::ReedsSheppStateSpace* RSspace = new ompl::base::ReedsSheppStateSpace(2.7);
        ompl::base::StateSpacePtr space(RSspace);
        ompl::base::ScopedState<ompl::base::SE2StateSpace> RSstart(space);
        RSstart->setXY(rrtPath[rrtPath.size() - 1]->pose.x(), rrtPath[rrtPath.size() - 1]->pose.y());
        RSstart->setYaw(rrtPath[rrtPath.size() - 1]->theta);
        ompl::base::ScopedState<ompl::base::SE2StateSpace> RSgoal(space);
        RSgoal->setXY(goal.pose.position.x, goal.pose.position.y);
        RSgoal->setYaw(this->goalYaw);
        ompl::base::ScopedState<ompl::base::SE2StateSpace> tempState(space);
        for(int i = 0; i < 20; i++)
        {
            RSspace->interpolate(RSstart.get(), RSgoal.get(), 0.05 * i, tempState.get());
            rrtPath.push_back(new State({tempState->getX(), tempState->getY(), 0.0}, tempState->getYaw()));
        }

        ROS_ERROR("error 15");
        std::vector<geometry_msgs::PoseStamped> path;
        //Тут нужно проверить внесение нод
        for(State*& poi : rrtPath)
        {
            geometry_msgs::PoseStamped msgs;
            msgs.header = this->header;
            tf::Quaternion q;
            q.setEuler(poi->theta, 0, 0);
            tf::quaternionTFToMsg(q, msgs.pose.orientation);
            msgs.pose.position.x = poi->pose.x();
            msgs.pose.position.y = poi->pose.y();
            msgs.pose.position.z = poi->pose.z();
            path.push_back(msgs);
        }
        path[path.size() - 1].pose.orientation = goal.pose.orientation;
        return path;
        */
    }

    bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
    {
        this->ma.markers.clear();
        this->header = start.header;
        this->stamp = start;
        this->zpose = start.pose.position.z;
        ROS_INFO("starting t@#@#@#!");
        tf::Quaternion initRotation = tf::Quaternion(start.pq.x, start.pq.y, start.pq.z, start.pq.w);
        tf::Quaternion goalRotation = tf::Quaternion(goal.pq.x, goal.pq.y, goal.pq.z, goal.pq.w);
        double roll, pitch, yaw;
        double groll, gpitch, gyaw;

        tf::Matrix3x3(initRotation).getRPY(roll, pitch, yaw);
        tf::Matrix3x3(goalRotation).getRPY(groll, gpitch, gyaw);
        this->startYaw = yaw;
        this->goalYaw = gyaw;
        
        plan = this->createRRTPath(start, goal);
        plan[plan.size() - 1].pose.orientation = goal.pose.orientation;
        
        
        this->publisher.publish(this->ma);
        return true;
    }
}; // namespace rrt_planner