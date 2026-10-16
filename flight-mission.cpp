#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <fstream>
#include <vector>
#include <string>

struct Waypoint {
    std::string name;
    double x, y, z;
};

Waypoint drone_pose = {"Current", 0.0, 0.0, 0.0};
bool is_odom_ready = false;
double accumulated_path_length = 0.0;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (is_odom_ready) {
        double dx = msg->pose.pose.position.x - drone_pose.x;
        double dy = msg->pose.pose.position.y - drone_pose.y;
        double dz = msg->pose.pose.position.z - drone_pose.z;
        accumulated_path_length += std::hypot(std::hypot(dx, dy), dz); 
    }
    drone_pose.x = msg->pose.pose.position.x;
    drone_pose.y = msg->pose.pose.position.y;
    drone_pose.z = msg->pose.pose.position.z;
    is_odom_ready = true;
}

double calculate2DDistance(const Waypoint& p1, const Waypoint& p2) {
    return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

void sendFlightCommand(ros::Publisher& pub, const Waypoint& target) {
    geometry_msgs::PoseStamped cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "world";
    cmd.pose.position.x = target.x;
    cmd.pose.position.y = target.y;
    cmd.pose.position.z = target.z;
    cmd.pose.orientation.w = 1.0;
    
    pub.publish(cmd);
    ROS_INFO(">>> Dispatched command to Waypoint %s: [%.2f, %.2f, %.2f]", 
             target.name.c_str(), target.x, target.y, target.z);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mission_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ros::Subscriber odom_sub = nh.subscribe("/drone_0_visual_slam/odom", 10, odomCallback);
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

    ros::Rate loop_rate(10); 

    ROS_INFO("Initializing Mission... Waiting for telemetry (Point A).");
    while (ros::ok() && !is_odom_ready) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    std::vector<Waypoint> route(5);
    
    route[0] = {"A", drone_pose.x, drone_pose.y, drone_pose.z}; 
    ROS_INFO("Home Point (A) locked at spawn: [%.2f, %.2f, %.2f]", route[0].x, route[0].y, route[0].z);

    route[1].name = "B";
    route[2].name = "C";
    route[3].name = "D";

    bool load_status = true;
    load_status &= nh_private.getParam("B_x", route[1].x);
    load_status &= nh_private.getParam("B_y", route[1].y);
    load_status &= nh_private.getParam("B_z", route[1].z);
    load_status &= nh_private.getParam("C_x", route[2].x);
    load_status &= nh_private.getParam("C_y", route[2].y);
    load_status &= nh_private.getParam("C_z", route[2].z);
    load_status &= nh_private.getParam("D_x", route[3].x);
    load_status &= nh_private.getParam("D_y", route[3].y);
    load_status &= nh_private.getParam("D_z", route[3].z);

    if (!load_status) {
        ROS_FATAL("Failed to load YAML coordinates! Ensure B, C, D are defined.");
        ros::shutdown(); 
        return 1;
    }

    route[4] = {"A (Return)", route[0].x, route[0].y, route[0].z};

    std::ofstream logger("flight_log.csv");
    logger << "Timestamp_sec,Drone_X,Drone_Y,Drone_Z,Target_Name,Distance_to_Target\n";

    int target_idx = 1; 
    bool is_flying_to_target = false;
    bool is_hovering = false;
    ros::Time hover_start_time;
    
    ros::Time mission_start = ros::Time::now();
    ros::Time leg_start = mission_start;
    std::vector<double> leg_durations;

    while (ros::ok() && target_idx < 5) {
        ros::spinOnce();

        Waypoint active_target = route[target_idx];
        double distance_left = calculate2DDistance(drone_pose, active_target);

        if (!is_flying_to_target) {
            sendFlightCommand(goal_pub, active_target);
            is_flying_to_target = true;
            is_hovering = false;
        }

        ROS_INFO_THROTTLE(1.0, "[Nav] Distance to %s: %.2f m", active_target.name.c_str(), distance_left);

        if (distance_left < 1.5) {
            if (!is_hovering) {
                is_hovering = true;
                hover_start_time = ros::Time::now();
            } else if ((ros::Time::now() - hover_start_time).toSec() >= 1.0) {
                double leg_time = (ros::Time::now() - leg_start).toSec();
                leg_durations.push_back(leg_time);
                ROS_INFO("+++ Waypoint %s verified! (Time: %.2f s) +++", active_target.name.c_str(), leg_time);
                
                target_idx++;
                is_flying_to_target = false; 
                leg_start = ros::Time::now();
            }
        } else {
            is_hovering = false; 
        }

        double t_elapsed = (ros::Time::now() - mission_start).toSec();
        logger << t_elapsed << "," 
               << drone_pose.x << "," << drone_pose.y << "," << drone_pose.z << "," 
               << active_target.name << "," << distance_left << "\n";
        
        logger.flush();

        loop_rate.sleep();
    }

    logger.close();

    ROS_INFO("==============================");
    ROS_INFO("MISSION ACCOMPLISHED (Returned to Base)");
    ROS_INFO("Total Flight Time: %.2f sec", (ros::Time::now() - mission_start).toSec());
    ROS_INFO("Total Distance Flown: %.2f m", accumulated_path_length);
    for (size_t i = 0; i < leg_durations.size(); ++i) {
        ROS_INFO("Leg %s -> %s: %.2f sec", route[i].name.c_str(), route[i+1].name.c_str(), leg_durations[i]);
    }
    ROS_INFO("==============================");

    return 0;
}
