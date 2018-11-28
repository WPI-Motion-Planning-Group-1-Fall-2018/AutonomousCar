#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <prius_controller/PriusControllerConfig.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <prius_msgs/Control.h>
#include <prius_msgs/MotionPlanning.h>

namespace Prius {

class PriusController
{
public:

    PriusController(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~PriusController();

    void calculateAndPublishControls();
    bool m_received_msg = false;

private:

    //private functions
    void dynamicReconfigureCallback(prius_controller::PriusControllerConfig &config, uint32_t level);
    void motionPlanningCallback(const prius_msgs::MotionPlanning::ConstPtr &msg);
    void gazeboStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg);

    double calculateYawRate();
    void getCurrentCommandIterator();
    double calculateSpeed();
    void calculateControls();
    void calculateSteering();
    void calculateGear();
    void calculatePedals();
    void publishControls();
    void extractPriusPose();

    //subpubs

    ros::Subscriber motion_planning_sub;
    ros::Subscriber gazebo_state_sub;
    ros::Publisher control_pub;

    //dynamic reconfigure
    dynamic_reconfigure::Server<prius_controller::PriusControllerConfig> server;
    dynamic_reconfigure::Server<prius_controller::PriusControllerConfig>::CallbackType f;

    //messages
    prius_msgs::MotionPlanning m_mp_control;
    prius_msgs::Control m_control;
    gazebo_msgs::ModelStates m_model_states;
    gazebo_msgs::ModelState m_prius_state;


    //keep track of stuff and things
    double m_yaw_rate;
    double m_speed;
    int m_control_it = 0;
    ros::Time m_current_time;
    ros::Time m_start_time = ros::Time::now();
    ros::Duration m_d_time;


    //steering PID stuff
    ros::Time m_previous_time_s = ros::Time::now();
    ros::Duration dt_s;
    double m_prev_err_s = 0;
    double m_int_s = 0;
    double m_kp_s;
    double m_ki_s;
    double m_kd_s;

    //speed PID stuff
    ros::Time m_previous_time_p = ros::Time::now();
    ros::Duration dt_p;
    double prev_speed = 0;
    double m_prev_err_p = 0;
    double m_int_p = 0;
    double m_kp_p;
    double m_ki_p;
    double m_kd_p;
};
}
