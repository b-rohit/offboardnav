/**
 *  * @file offb_node.cpp
 *   * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 *    * stack and tested in Gazebo SITL
 *     */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <boost/thread/thread.hpp>

class autonomous {
    private:
    ros::Subscriber state_sub;
    ros::Subscriber pos_sub;
    ros::Subscriber vel_sub;
    ros::Publisher local_pos_pub;
    ros::Publisher local_vel_pub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    mavros_msgs::State current_state;
    float x;
    float y;
    float z;
    bool done;
    public:
    bool home;
    bool stop;
    autonomous(){
        ros::NodeHandle nh;
        state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &autonomous::state_cb, this);
        pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",10, &autonomous::localPos, this);
        vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity",10, &autonomous::localVel, this);
        local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
        local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
        arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
        set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
        try {
            boost::thread nav(boost::bind(&autonomous::navigate, this));
            ROS_INFO("thread started");
        }
        catch(...) {
            ROS_ERROR("thread not started");
        }
        x = y = 0;
        z = 0.5;
        done = false;
        home = false;
        stop = false;
    }

    void state_cb(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
        //ROS_INFO("curent state %d", current_state.connected);
    }

    void navigate(){
        ros::Rate rate(30.0);
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 1;
        for(int i = 100; ros::ok() && i > 0; --i){
            local_pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        geometry_msgs::TwistStamped twist;
        twist.twist.linear.x =0.0;
        twist.twist.linear.y =0.0;
        twist.twist.linear.z =0.5;
        ros::Time last_request = ros::Time::now();
        while(ros::ok()) {
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = z;
            if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.success){
                    ROS_INFO("Offboard enabled");
                }
                else
                    ROS_INFO("Offboard not successfull");
                last_request = ros::Time::now();
            } 
            else {
                if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
                    if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
                if(home && current_state.armed){
                    arm_cmd.request.value = false;
                    if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                        ROS_INFO("Vehicle unarmed");
                        break;
                    }
                }
            }

            local_pos_pub.publish(pose);
            ROS_INFO("published");
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("thread stopped");
        stop = true;
    }
   void setPos(float x, float y, float z, float delay = 0, bool wait = 1) {
        this->done = false;
        this->x = x;
        this->y = y;
        this->z = z;
        if (wait) {
            ros::Rate rate(30.0);
            while(!this->done && ros::ok() ){ 
                //ROS_INFO("waiting");
                ros::spinOnce();
                rate.sleep();
            }
        }
        //sleep(delay);
        return;
    }

    float absolute(float a){
        if(a<0)
            return -1*a;
        else
            return a;
    }

    bool hasReached(char ch, float pos1, float pos2){
       ROS_INFO("Position %c current %f target %f abs %f", ch, pos1, pos2, absolute(pos1- pos2));
       return absolute(pos1-pos2) < 0.1;
    }
    void localVel(geometry_msgs::TwistStamped twist){
       //ROS_INFO("vel x %f y %f z %f", twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z);
       return;
    }

    void localPos(geometry_msgs::PoseStamped pose){
       if(absolute(pose.pose.position.x -3)<0.1 && absolute(pose.pose.position.y -0)<0.1 && absolute(pose.pose.position.z -0)<0.1){
           ROS_INFO("home"); 
           home = true;
       }
       if(hasReached('X', pose.pose.position.x, this->x) && hasReached('Y', pose.pose.position.y, this->y) && hasReached('Z', pose.pose.position.z, this->z)){
            ROS_INFO("reached");
            done = true;
        }
        return;
    }
};

int  main(int argc, char **argv){
    ros::init(argc, argv, "offnav");
    //the setpoint publishing rate MUST be faster than 2Hzros::Time last_request = ros::Time::now();

    autonomous auton;
    ros::Time start = ros::Time::now();
    //ROS_INFO("durationi %d", ros::Duration(1.0));
    while (!auton.stop) {
        ROS_INFO("climb");
        auton.setPos(0.0, 0.0, 0.5, 0);
        //setpoint.set(0.0, 0.0, 10.0, 5);
        ROS_INFO("right");
        auton.setPos(0.5, 0.0, 0.5 ,0);

        ROS_INFO("descend");
        auton.setPos(0.5, 0.0, 0.0 ,0);
    }
    ros::shutdown();
    return 1;
}
