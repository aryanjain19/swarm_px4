#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <five_drone/Data.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Float64.h>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped pose;
const int drone = 1;
int initial_x = 1, initial_y = 0, initial_z = 2;
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
    }
    res.success=true;
    return true;
}

void leader_callback(five_drone::Data msg)
{
    // data = msg.message.data;
    leader = msg.a;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hover_1");
    ros::NodeHandle nh;
    // ros::NodeHandle nh_private("~");

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/uav1/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav1/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav1/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav1/mavros/set_mode");

    ros::ServiceServer server=nh.advertiseService("make_leader",callback);
    ros::ServiceClient client=nh.serviceClient<std_srvs::SetBool>("make_leader");

    ros::Publisher pub = nh.advertise<five_drone::Data>("/leader_who",10);
    ros::Subscriber leader_sub = nh.subscribe<five_drone::Data>
            ("/leader_who", 10, leader_callback);

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


        local_pos_pub.publish(pose);

        if(leader == drone || leader == -1)
        pub.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}