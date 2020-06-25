#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>

namespace utils {
    visualization_msgs::Marker createLine(tf::Vector3 start, tf::Vector3 end, tf::Vector3 color, int id, float alpha = 1.0)
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
        line.color.r = color.x();
        line.color.g = color.y();
        line.color.b = color.z();
        line.color.a = alpha;
        geometry_msgs::Point lineStart, lineEnd;
        lineStart.x = start.x();
        lineStart.y = start.y();
        lineStart.z = start.z();
        lineEnd.x = end.x();
        lineEnd.y = end.y();
        lineEnd.z = end.z();
        line.points.push_back(lineStart);
        line.points.push_back(lineEnd);
        return line;
    }

    visualization_msgs::Marker createVizPoint(tf::Vector3 pos, tf::Vector3 color, int id, float alpha = 1.0)
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
        point.color.r = color.x();
        point.color.g = color.y();
        point.color.b = color.z();
        point.color.a = alpha;
        geometry_msgs::Pose p;
        p.position.x = pos.x();
        p.position.y = pos.y();
        p.position.z = pos.z();
        p.orientation.w = 1.0;
        point.pose = p;
        return point;
    }

    visualization_msgs::Marker createVizCylinder(tf::Vector3 pos, tf::Vector3 color, float radius, int id, float alpha = 1.0)
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
        point.color.r = color.x();
        point.color.g = color.y();
        point.color.b = color.z();
        point.color.a = alpha;
        geometry_msgs::Pose p;
        p.position.x = pos.x();
        p.position.y = pos.y();
        p.position.z = pos.z();
        p.orientation.w = 1.0;
        point.pose = p;
        return point;
    }


}