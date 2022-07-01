#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <five_drone/Data.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/NavSatFix.h>
#include <string>
#include <geographic_msgs/GeoPoseStamped.h>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped pose, pose_data, leader_pose;
// sensor_msgs::NavSatFix leader_pos_data;
// geographic_msgs::GeoPoseStamped tar_pos;
float pos[3],leader_pos[3];
const int drone = 2;
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
        flag = true;

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
        msg.x = 0.0;
        msg.y = 0.0;
        msg.z = 0.0;
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
    leader_pos[0] = msg.x;
    leader_pos[1] = msg.y;
    leader_pos[2] = msg.z;
}

void pos_sub_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    
    pose_data = *msg;
    pos[0] = pose_data.pose.position.x;
    pos[1] = pose_data.pose.position.y;
    pos[2] = pose_data.pose.position.z;

    // ROS_INFO("%f %f %f",pos[0],pos[1],pos[2]);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hover_2");
    ros::NodeHandle nh;
    // ros::NodeHandle nh_private("~");

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/uav2/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav2/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav2/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav2/mavros/set_mode");

    ros::ServiceServer server=nh.advertiseService("make_leader",callback);
    ros::ServiceClient client=nh.serviceClient<std_srvs::SetBool>("make_leader");

    ros::Publisher pub = nh.advertise<five_drone::Data>("/leader_who",10);
    ros::Subscriber leader_sub = nh.subscribe<five_drone::Data>
            ("/leader_who", 10, leader_callback);

    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/uav2/mavros/local_position/pose", 10, pos_sub_callback);
    
    // ros::Publisher global_pos_pub = nh.advertise<geographic_msgs::GeoPoseStamped>
    //         ("/uav0/mavros/setpoint_position/global", 10);
    

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
    msg.x = 0.0;
    msg.y = 0.0;
    msg.z = 0.0;

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


        // if(flag==false)
        local_pos_pub.publish(pose);

        if(leader == -1)
        {
            // pub.publish(msg);
            local_pos_pub.publish(pose);
        }

        if(leader == drone)
        {            
            msg.x = pos[0];
            msg.y = pos[1];
            msg.z = pos[2];    
            pub.publish(msg);
        }

        if(leader != drone && leader != -1)
        {
            leader_pose.pose.position.x = leader_pos[0];
            leader_pose.pose.position.y = leader_pos[1];
            leader_pose.pose.position.z = leader_pos[2]-1;

            // ROS_INFO("Leader coords = %f %f %f",leader_pos[0],leader_pos[1],leader_pos[2]);

            local_pos_pub.publish(leader_pose);
        }
        

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}