#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <five_drone/Data.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/CommandCode.h>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped pose;
mavros_msgs::WaypointPush wp_push_srv; // List of Waypoints
mavros_msgs::Waypoint wp;
const int drone = 0;
int initial_x = 0, initial_y = 0, initial_z = 2;
bool flag=false;
five_drone::Data msg;
float dif_x,dif_y,dif_z;
int leader = -1;

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

bool callback(std_srvs::SetBool::Request &req,std_srvs::SetBool::Response &res)
{
    if (req.data==true)
    {
        leader = drone;
        res.message="service true postive";

        msg.message.data = "Leader is";
        msg.a = leader;
        // pub.publish(msg);

        pose.pose.position.x = initial_x;
        pose.pose.position.y = initial_y;
        pose.pose.position.z = initial_z+1;

        
        // WP 0
        wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        wp.command        = mavros_msgs::CommandCode::NAV_TAKEOFF;
        wp.is_current     = true;
        wp.autocontinue   = true;
        wp.x_lat          = 47.3978206;
        wp.y_long         = 8.543987;
        wp.z_alt          = 545.26191412;
        wp_push_srv.request.waypoints.push_back(wp);
        // WP 1
        wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        wp.command        = mavros_msgs::CommandCode::NAV_LOITER_TIME;
        wp.is_current     = false;
        wp.autocontinue   = true;
        wp.x_lat          = 49.3962527;
        wp.y_long         = 8.5467917;
        wp.z_alt          = 545.26191412;
        wp.param1			= 10;
        wp.param3			= 2;
        wp.param4			= 1;
        wp_push_srv.request.waypoints.push_back(wp);
        
        // WP 2
        wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        wp.command        = mavros_msgs::CommandCode::NAV_WAYPOINT;
        wp.is_current     = false;
        wp.autocontinue   = true;
        wp.x_lat          = 47.3977783;
        wp.y_long         = 10.547906;
        wp.z_alt          = 545.26191412;
        wp_push_srv.request.waypoints.push_back(wp);

        // WP 3
        wp.frame          = mavros_msgs::Waypoint::FRAME_MISSION;
        wp.command        = mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH;
        wp.is_current     = false;
        wp.autocontinue   = true;
        wp.x_lat          = 47.3977783;
        wp.y_long         = 8.547906;
        wp.z_alt          = 538.26191412;
        wp_push_srv.request.waypoints.push_back(wp);

        flag = true;
        
    }
    else
    {
        leader = -1;
        res.message="service true neg";

        msg.message.data = "Leader is";
        msg.a = leader;
        // pub.publish(msg);

        pose.pose.position.x = initial_x;
        pose.pose.position.y = initial_y;
        pose.pose.position.z = initial_z;

        flag=false;
    }
    res.success=true;
    ROS_INFO("rosservice was called");

    return true;
}

void leader_callback(five_drone::Data msg)
{
    // data = msg.message.data;
    leader = msg.a;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hover_0");
    ros::NodeHandle nh;
    // ros::NodeHandle nh_private("~");

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/uav0/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav0/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav0/mavros/set_mode");

    ros::ServiceServer server=nh.advertiseService("make_leader",callback);
    ros::ServiceClient client=nh.serviceClient<std_srvs::SetBool>("make_leader");

    ros::Publisher pub = nh.advertise<five_drone::Data>("/leader_who",10);
    ros::Subscriber leader_sub = nh.subscribe<five_drone::Data>
            ("/leader_who", 10, leader_callback);

    ros::ServiceClient wp_client = nh.serviceClient<mavros_msgs::WaypointPush>
            ("mavros/mission/push");
    mavros_msgs::SetMode auto_set_mode;
    auto_set_mode.request.custom_mode = "AUTO.MISSION";


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // geometry_msgs::PoseStamped pose;
    pose.pose.position.x = initial_x;
    pose.pose.position.y = initial_y;
    pose.pose.position.z = initial_z;

    // intial service message
    msg.message.data = "Leader is";
    msg.a = -1;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }


        if(flag==false)
        local_pos_pub.publish(pose);

        if(leader == drone || leader == -1)
        pub.publish(msg);

        if(leader == drone)
        {
            if (wp_client.call(wp_push_srv)) 
            {
                ROS_INFO("Send waypoints ok: %d", wp_push_srv.response.success);
                if (current_state.mode != "AUTO.MISSION") {
                    if( set_mode_client.call(auto_set_mode) &&
                        auto_set_mode.response.mode_sent){
                        ROS_INFO("AUTO.MISSION enabled");
                    }
                }
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}