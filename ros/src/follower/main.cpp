#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
#include <geometry_msgs/Twist.h>

#include <cmath>

#define MOVE_FOLLOWER_SPEED   1
#define ROTATE_FOLLOWER_SPEED 4
#define EPS 1e-2

const double PI = 3.14159265;

turtlesim::Pose pose1;
turtlesim::Pose pose2;

ros::Publisher pub2;

void move_follower() {
    geometry_msgs::Twist move;

    float x = pose1.x - pose2.x;
    float y = pose1.y - pose2.y;

    if (x*x + y*y < EPS) {
        pub2.publish(move);
        return;
    }

    move.linear.x = MOVE_FOLLOWER_SPEED;

    float need_angle = std::atan2(y, x);
    float have_angle = pose2.theta;
    while (std::abs(have_angle) > PI + EPS) {
        if (have_angle > 0)
            have_angle -= 2 * PI;
        else
            have_angle += 2 * PI;
    }

    float rotate_angle = (need_angle - have_angle) * ROTATE_FOLLOWER_SPEED;
    if (std::abs(rotate_angle) < EPS) {
        rotate_angle = 0;
    }

    move.angular.z = rotate_angle;

    pub2.publish(move);
}

void turtle1_pose_callback(const turtlesim::Pose& pos) {
    pose1 = pos;
    move_follower();
}

void follower_pose_callback(const turtlesim::Pose& pos) {
    pose2 = pos;
}

void spawn_follower(ros::NodeHandle& node) {
     ros::ServiceClient client = node.serviceClient<turtlesim::Spawn>("spawn");

     turtlesim::Spawn srv;
     srv.request.x = 2;
     srv.request.y = 2;
     srv.request.theta = 0.5;
     srv.request.name = "follower";

     client.call(srv);
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "turtle_follower");

    ros::NodeHandle node;

    spawn_follower(node);

    ros::Subscriber sub1 = node.subscribe("/turtle1/pose", 1, turtle1_pose_callback);
    ros::Subscriber sub2 = node.subscribe("/follower/pose", 1, follower_pose_callback);
    pub2 = node.advertise<geometry_msgs::Twist>("/follower/cmd_vel", 10);

    ros::Rate loop_rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
