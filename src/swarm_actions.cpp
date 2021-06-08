/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Unbounded Robotics Inc.
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ghufran Ullah */

#include <memory>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <control_toolbox/pid.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <string>
#include <cstdlib>
#include <swarm_actions/swarm_actions.h>

SwarmActions::SwarmActions(const std::string& name) : 
    action_name_(name), 
    action_server_(nh_, name, boost::bind(&SwarmActions::executeCb, this, _1), false)
{
}

void SwarmActions::init(const std::string& ns) {
    ros::NodeHandle p_nh("~");
    p_nh.getParam("/" + ns + "/x", initial_x);
    p_nh.getParam("/" + ns + "/y", initial_y);
    p_nh.getParam("/" + ns + "/z", initial_z);
    p_nh.getParam("/" + ns + "/R", initial_R);
    p_nh.getParam("/" + ns + "/P", initial_P);
    p_nh.getParam("/" + ns + "/Y", initial_Y);

    std::cout<<"Robot: "<<ns<<std::endl;
    std::cout<<"initial_x: "<<initial_x<<std::endl;
    std::cout<<"initial_y: "<<initial_y<<std::endl;
    std::cout<<"initial_z: "<<initial_z<<std::endl;
    std::cout<<"initial_R: "<<initial_R<<std::endl;
    std::cout<<"initial_P: "<<initial_P<<std::endl;
    std::cout<<"initial_Y: "<<initial_Y<<std::endl;
    // setpoint publishing rate MUST be faster than 2Hz. From mavros documentation
    double rate;
    p_nh.param("rate", rate, 100.0);
    rate_ = ros::Rate(rate);
    cycle_time_ = rate_.expectedCycleTime().toSec();

    std::string control_mode;
    p_nh.param<std::string>("control_mode", control_mode, "position");

    // check the topic it needs to subscribe and fix it
    local_cmd_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(std::string("/") + ns + std::string("/mavros/setpoint_position/local"), 5);
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(std::string("/") + ns + std::string("/mavros/cmd/arming"));
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(std::string("/") + ns + std::string("/mavros/set_mode"));
    
    state_sub_ = nh_.subscribe<mavros_msgs::State>(std::string("/") + ns + std::string("/mavros/state"), 10, &SwarmActions::stateCb, this);
    local_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(std::string("/") + ns + std::string("/mavros/local_position/pose"), 10, &SwarmActions::poseCb, this);
    // start the action server
    ROS_INFO("Running the Action Server...");
    action_server_.start();
}

void SwarmActions::idle() {
    while(ros::ok()){
        ROS_INFO("Action Server Idle");
        ros::spinOnce();
        ROS_INFO("Action Server Idle");
        ros::Rate(5).sleep();
    }
}

void SwarmActions::executeCb(const GoalPtr &goal) {
    ROS_INFO("executeCB");
    float treshold =0.1;
    geometry_msgs::PoseStamped goal_to_reach = goal->goal;
    goal_to_reach.pose.position.x = goal_to_reach.pose.position.x - initial_x;
    goal_to_reach.pose.position.y = goal_to_reach.pose.position.y - initial_y;
    goal_to_reach.pose.position.z = goal_to_reach.pose.position.z - initial_z;

        auto success = true;
        // check for FCU connection
        if (current_state_.connected) {
            // Arm vehicle
            if (!current_state_.armed){
                ROS_INFO("Vehicle is not armed");
                action_server_.setAborted();
                success = false;
            }

            for (int i = 100; ros::ok() && i > 0; --i){
            local_cmd_pose_pub_.publish(goal_to_reach);
            ros::spinOnce();
            ros::Rate(20).sleep();
            }
            
            mavros_msgs::SetMode offb_set_mode;
            offb_set_mode.request.custom_mode = "OFFBOARD";
            if (!set_mode_client_.call(offb_set_mode))
                action_server_.setAborted();
            ros::spinOnce();

            auto time = 0.0;
            geometry_msgs::PoseStamped pose_diff;
            geometry_msgs::PoseStamped cmd_pose;
            while (ros::ok() && success == true) {
                ROS_INFO("in while loop");
                if(action_server_.isPreemptRequested() || !ros::ok()) {
                    action_server_.setPreempted();
                    success = false;
                    ROS_INFO("Preempting requested by client");
                    break;
                }
                // if (!set_offboard_client_.call(offb_set_mode)){
                //     action_server_list_[i].setAborted();
                //     success = false;
                //     break;
                // }
                pose_diff.pose.position.x = abs(current_pose_.pose.position.x - goal_to_reach.pose.position.x);
                pose_diff.pose.position.y = abs(current_pose_.pose.position.y - goal_to_reach.pose.position.y);
                pose_diff.pose.position.z = abs(current_pose_.pose.position.z - goal_to_reach.pose.position.z);
                std::cout<<"current_pose x: "<< current_pose_.pose.position.x <<std::endl;
                std::cout<<"current_pose y: "<< current_pose_.pose.position.y <<std::endl;
                std::cout<<"goal+pose_x :  "<< goal_to_reach.pose.position.x<<std::endl;
                std::cout<<"goal+pose_y :  "<< goal_to_reach.pose.position.y<<std::endl;
                if(pose_diff.pose.position.x <= (0.0 + treshold) && pose_diff.pose.position.y <= (0.0 + treshold)){
                    success = true;
                    break;
                }
                cmd_pose = current_pose_;
                ROS_INFO_ONCE("In the while loop");
                cmd_pose.header.stamp = ros::Time::now();
                local_cmd_pose_pub_.publish(goal_to_reach);
                feedback_.current_pose = cmd_pose;
                action_server_.publishFeedback(feedback_);
                time += cycle_time_;
                rate_.sleep();
                ros::spinOnce();
            }
        } 
        else {
            ROS_WARN("Mavros not connected to FCU.");
            action_server_.setAborted();
            success = false;
        }
        if(success)
        {   ROS_INFO("Success!");
            result_.error_code = Result::SUCCESSFUL;
            // set the action server to succeeded
            action_server_.setSucceeded(result_);
        }
}

void SwarmActions::stateCb(const mavros_msgs::State::ConstPtr& msg) {
  current_state_ = *msg;
}

void SwarmActions::poseCb(const geometry_msgs::PoseStamped::ConstPtr& current_pose) {
    current_pose_ = *current_pose;
    // current_pose_.pose.position.x = current_pose_.pose.position.x + initial_x;
    // current_pose_.pose.position.y = current_pose_.pose.position.y + initial_y;
    // current_pose_.pose.position.z = current_pose_.pose.position.z + initial_z;
}

int main(int argc, char** argv)
{ 
    ROS_INFO("main");
    ros::init(argc, argv, "SwarmActions");

    SwarmActions action_server0("/uav0/SwarmActions");
    action_server0.init("uav0");
    
    SwarmActions action_server1("/uav1/SwarmActions");
    action_server1.init("uav1");
    
    SwarmActions action_server2("/uav2/SwarmActions");
    action_server2.init("uav2");
    
    ros::spin();
    return 0;
}