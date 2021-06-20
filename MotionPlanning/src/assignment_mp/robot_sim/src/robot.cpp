#include "robot_sim/robot.h"

namespace robot_sim {

Robot::Robot(const std::map<std::string, size_t> &name_map)
{
  name_map_ = name_map;
  num_joints_ = name_map_.size();
  joint_names_.clear();
  for (std::map<std::string,size_t>::iterator it=name_map_.begin(); it!=name_map_.end(); ++it)
    joint_names_.push_back(it->first);
  joint_values_.resize(num_joints_, 0.0);
  joint_velocities_.resize(num_joints_, 0.0);
  goal_values_.resize(num_joints_, 0.0);
  max_jump_ = 1.0e-3;
  last_velocity_command_ = ros::Time(0);
  velocity_timeout_ = 0.5;
}

bool Robot::init()
{
  running_ = true;
  velocity_mode_ = false;
  last_tick_ = ros::Time::now();
  thread_ = boost::thread(boost::bind(&Robot::update, this));
  return true;
}

void Robot::stop()
{
  running_ = false;
  thread_.join();
}

std::vector<std::string> Robot::getJointNames() const
{
  return joint_names_;
}

std::vector<double> Robot::getJointValues() const
{
  mutex_.lock();
  std::vector<double> vals = joint_values_;
  mutex_.unlock();
  return vals;
}

std::vector<double> Robot::getJointValues(const std::vector<std::string> &names) const
{
  mutex_.lock();
  std::vector<double> vals(names.size(),0.0);
  for (size_t i=0; i<names.size(); i++)
  {
    std::map<std::string, size_t>::const_iterator it = name_map_.find(names[i]);
    if (it == name_map_.end())
    {
      ROS_ERROR_STREAM("Requested joint " << names[i] << " not found in robot");
    } 
    else
    {
      assert(it->second < joint_values_.size());
      vals[i] = joint_values_[it->second];
    }
  }
  mutex_.unlock();
  return vals;
}

void Robot::setTargetValues(const std::vector<double> &vals)
{
  assert(vals.size() == joint_values_.size());
  mutex_.lock();
  goal_values_ = vals;
  velocity_mode_ = false;
  mutex_.unlock(); 
}

void Robot::setJointValues(const std::vector<double> &vals)
{
  assert(vals.size() == joint_values_.size());
  mutex_.lock();
  goal_values_ = vals;
  joint_values_ = vals;
  velocity_mode_ = false;
  mutex_.unlock(); 
}

void Robot::setJointValues(const std::vector<std::string> &names, const std::vector<double> &vals)
{
  assert(names.size() == vals.size());
  mutex_.lock();
  for (size_t i=0; i<names.size(); i++)
  {
    std::map<std::string, size_t>::iterator it = name_map_.find(names[i]);
    if (it == name_map_.end())
    {
      ROS_ERROR_STREAM("Commanded joint " << names[i] << " not found in robot");
    } 
    else
    {
      assert(it->second < joint_values_.size());
      joint_values_[it->second] = vals[i];
      goal_values_[it->second] = vals[i];
    }
  }
  velocity_mode_ = false;
  mutex_.unlock();
}

void Robot::setTargetValues(const std::vector<std::string> &names, const std::vector<double> &vals)
{
  assert(names.size() == vals.size());
  mutex_.lock();
  for (size_t i=0; i<names.size(); i++)
  {
    std::map<std::string, size_t>::iterator it = name_map_.find(names[i]);
    if (it == name_map_.end())
    {
      ROS_ERROR_STREAM("Commanded joint " << names[i] << " not found in robot");
    } 
    else
    {
      assert(it->second < goal_values_.size());
      goal_values_[it->second] = vals[i];
    }
  }
  velocity_mode_ = false;
  mutex_.unlock();
}

void Robot::setVelocities(const std::vector<std::string> &names, const std::vector<double> &vels)
{
  assert(vels.size() == names.size());
  mutex_.lock();  
  for (size_t i=0; i<names.size(); i++)
  {
    std::map<std::string, size_t>::iterator it = name_map_.find(names[i]);
    if (it == name_map_.end())
    {
      ROS_ERROR_STREAM("Commanded joint " << names[i] << " not found in robot");
    } 
    else
    {
      assert(it->second < joint_velocities_.size());
      joint_velocities_[it->second] = vels[i];
    }
  }
  velocity_mode_ = true;
  last_velocity_command_ = ros::Time::now();
  mutex_.unlock();
}

  void Robot::setVelocities(const std::vector<double> &vels)
{
  assert(vels.size() == joint_values_.size());
  mutex_.lock();
  joint_velocities_ = vels;
  velocity_mode_ = true;
  last_velocity_command_ = ros::Time::now();
  mutex_.unlock();
}

void Robot::update()
{
  while(running_)
  {
    ros::Time now_tick = ros::Time::now();
    mutex_.lock();
    if (velocity_mode_)
    {
      double secs_since_command = (now_tick - last_velocity_command_).toSec();
      if (secs_since_command < velocity_timeout_)
      {
	double secs = (now_tick - last_tick_).toSec();
	for (size_t i=0; i<num_joints_; i++)
	{
	  joint_values_[i] += secs * joint_velocities_[i];
	}
      }
    }
    else
    {
      for (size_t i=0; i<num_joints_; i++)
      {
	double jump = goal_values_[i] - joint_values_[i];
	jump = std::max(jump, -max_jump_);
	jump = std::min(jump,  max_jump_);
	joint_values_[i] += jump;
      }
    }
    mutex_.unlock();
    last_tick_ = now_tick;
    boost::this_thread::sleep(boost::posix_time::milliseconds(1));
  }
}

} //namespace robot_sim
