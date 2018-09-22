#include <controller.hpp>

namespace Prius {

PriusController::PriusController(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
  motion_planning_sub = nh.subscribe<prius_msgs::MotionPlanning>("/mp", 100, &PriusController::motionPlanningCallback, this);

  gazebo_state_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 100, &PriusController::gazeboStatesCallback, this);

  control_pub = pnh.advertise<prius_msgs::Control>("/prius/control", 100);

  pnh.getParam("kp_s", m_kp_s);

  pnh.getParam("ki_s", m_ki_s);

  pnh.getParam("kd_S", m_kd_s);

  pnh.getParam("kp_p", m_kp_p);

  pnh.getParam("ki_p", m_ki_p);

  pnh.getParam("kd_p", m_kd_p);
}

PriusController::~PriusController(){}

void PriusController::calculateAndPublishControls()
{
  calculateYawRate();

  calculateControls();

  publishControls();
}

void PriusController::calculateControls()
{
  calculateSteering();

  calculateGear();

  calculatePedals();
}

void PriusController::calculateSteering()
{
  dt_s = ros::Time::now() - m_previous_time_s;

  auto yaw_rate = calculateYawRate();

  double error = m_mp_control.yaw_rate - yaw_rate;

  m_int_s += error * dt_s.toSec();

  double derivative = (error - m_prev_err_s) / dt_s.toSec();

  m_control.steer = m_kp_s * error + m_ki_s * m_int_s + m_kd_s * derivative;

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
  if(m_mp_control.speed > 0)
  {
    m_control.shift_gears = prius_msgs::Control::FORWARD;
  }

  if(m_mp_control.speed < 0)
  {
    m_control.shift_gears = prius_msgs::Control::REVERSE;
  }
}

void PriusController::calculatePedals()
{
  dt_p = ros::Time::now() - m_previous_time_p;

  auto speed = calculateSpeed();

  double error = m_mp_control.speed - speed;

  m_int_p += error * dt_p.toSec();

  double derivative = (error - m_prev_err_p) / dt_p.toSec();

  double control_speed = m_kp_p * error + m_ki_p * m_int_p + m_kd_p *  derivative;

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
    m_control.brake = control_speed;

    m_control.throttle = 0;
  }
}

void PriusController::publishControls()
{
  control_pub.publish(m_control);
}

double PriusController::calculateYawRate()
{
  return m_prius_state.twist.angular.z;
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

  calculateAndPublishControls();
}

void PriusController::gazeboStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
  m_model_states = *msg;

  extractPriusPose();
}


}
