#include <controller.hpp>

namespace Prius {

PriusController::PriusController(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    ROS_INFO_STREAM("Subscribing to /mp");
    motion_planning_sub = nh.subscribe<prius_msgs::MotionPlanning>("/mp", 100, &PriusController::motionPlanningCallback, this);
    ROS_INFO_STREAM("Subscribing to /gazebo/model_states");
    gazebo_state_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 100, &PriusController::gazeboStatesCallback, this);
    ROS_INFO_STREAM("Publishing to /prius/control");
    control_pub = pnh.advertise<prius_msgs::Control>("/prius/control", 100);
    ROS_INFO_STREAM("Setting Up Dynamic Reconfigure Server");
    f = boost::bind(&PriusController::dynamicReconfigureCallback, this, _1, _2);
    server.setCallback(f);
}

PriusController::~PriusController()
{
    ROS_INFO_STREAM("PriusController Destructor Called");
}

void PriusController::dynamicReconfigureCallback(prius_controller::PriusControllerConfig &config, uint32_t level)
{
    ROS_INFO_STREAM("Dynamic Reconfigure Updated");
    m_kp_s = config.kp_s;
    m_ki_s = config.ki_s;
    m_kd_s = config.kd_s;
    m_kp_p = config.kp_p;
    m_ki_p = config.ki_p;
    m_kd_p = config.kd_p;
}

void PriusController::calculateAndPublishControls()
{
    calculateControls();
    publishControls();
}

void PriusController::calculateControls()
{
    getCurrentCommandIterator();
    calculateSteering();
    calculateGear();
    calculatePedals();
}

void PriusController::calculateSteering()
{
    dt_s = ros::Time::now() - m_previous_time_s;
    m_yaw_rate = calculateYawRate();
    double error = m_mp_control.yaw_rates[m_control_it] - m_yaw_rate;
    m_int_s += error * dt_s.toSec();
    double derivative = (error - m_prev_err_s) / dt_s.toSec();
    m_control.steer = m_kp_s * error + m_ki_s * m_int_s - m_kd_s * derivative;
    if(m_control.steer > 1)
    {
        m_control.steer = 1;
    }

    if(m_control.steer < -1)
    {
        m_control.steer = -1;
    }

    m_previous_time_s = ros::Time::now();
    m_prev_err_s = error;
}

void PriusController::calculateGear()
{
    if(m_speed > 0)
    {
        m_control.shift_gears = prius_msgs::Control::FORWARD;
    }

    if(m_speed < 0)
    {
        m_control.shift_gears = prius_msgs::Control::REVERSE;
    }
}

void PriusController::calculatePedals()
{
    dt_p = ros::Time::now() - m_previous_time_p;
    m_speed = m_mp_control.speeds[m_control_it];
    auto speed = calculateSpeed();
    if(speed > m_mp_control.max_speed)
    {
        speed = m_mp_control.max_speed;
    }

    double set_point;
    if(m_speed > speed)
    {
        set_point = speed + m_mp_control.max_accel * dt_p.toSec();
    }

    if(m_speed < speed)
    {
        set_point = speed - m_mp_control.max_accel * dt_p.toSec();
    }

    else
    {
        set_point = m_speed;
    }

    double error = set_point - speed;
    m_int_p += error * dt_p.toSec();
    double derivative = (error - m_prev_err_p) / dt_p.toSec();
    double control_speed = m_kp_p * error + m_ki_p * m_int_p - m_kd_p *  derivative;

    if(control_speed > 1)
    {
        control_speed = 1;
    }

    if(control_speed < -1)
    {
        control_speed = -1;
    }

    if(error >= 0)
    {
        m_control.throttle = control_speed;
        m_control.brake = 0;
    }

    if(error < 0)
    {
        m_control.brake = -control_speed;
        m_control.throttle = 0;
    }

    m_previous_time_p = ros::Time::now();
    m_prev_err_p = error;
}

void PriusController::publishControls()
{
    control_pub.publish(m_control);
}

double PriusController::calculateYawRate()
{
    return m_prius_state.twist.angular.z;
}

void PriusController::getCurrentCommandIterator()
{
  m_current_time = ros::Time::now();
  m_d_time = m_current_time - m_previous_time;
  m_previous_time = m_current_time;
  double duration = m_mp_control.durations[m_control_it] / 1000;
  if(m_d_time.toSec() >= duration)
  {
    m_control_it++;
  }
  if(m_control_it > m_mp_control.yaw_rates.size())
  {
    ROS_ERROR("Reached end of profile without receiving new commands.... good luck");
  }
}

double PriusController::calculateSpeed()
{
    return m_prius_state.twist.linear.x;
}

void PriusController::extractPriusPose()
{
    for(int i = 0; i < m_model_states.name.size(); i++)
    {
        if(m_model_states.name[i] == "prius")
        {
            m_prius_state.twist = m_model_states.twist[i];
            m_prius_state.pose = m_model_states.pose[i];
            break;
        }
    }
}

void PriusController::motionPlanningCallback(const prius_msgs::MotionPlanning::ConstPtr &msg)
{
    m_mp_control = *msg;
    m_control_it = 0;
    calculateAndPublishControls();
}

void PriusController::gazeboStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    m_model_states = *msg;
    extractPriusPose();
}
}
