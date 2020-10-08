/**
 * @file main.cpp
 * @author Suhan Park (psh117@snu.ac.kr)
 * @brief Panda Motion Generator for Calibration (joint position controlled mode)
 * @version 0.1
 * @date 2020-10-07
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include <cmath>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>

#include <Eigen/Dense>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

typedef Eigen::Matrix<double, 7, 1> Vector7d;
class CSVJointPosReader
{
public:
  CSVJointPosReader(const std::string & file_name) : input_file_(file_name)
  {
    if(!input_file_.is_open()) throw std::runtime_error("Could not open file");
    std::string line, colname;

    // Read the column names
    // Read data, line by line

    while(std::getline(input_file_, line, ','))
    {
      std::string name = line;
      std::getline(input_file_, line);
      
      std::stringstream ss(line);
      
      // Extract each integer
      Vector7d pos;
      for (int i=0; i<7; i++)
      {
        if(ss.peek() == ',') ss.ignore();
        ss >> pos(i);
      }
      pos = pos / 180.0 * M_PI; 
      q_data_.push_back(std::make_pair(name,pos));
    }
    
  }
  const Vector7d & getJointData(int index)
  {
    return q_data_[index].second;
  }
  
  const std::vector<std::pair<std::string, Vector7d>>& getJointDatas()
  {
    return q_data_;
  }

  void print()
  {
    std::cout << "[CSVJointPosReader] Read results" << std::endl;
    for (auto & q : q_data_)
    {
      std::cout << "    name: " << q.first << std::endl;
      std::cout << "    qval: " << q.second.transpose() << std::endl << std::endl;
    }
  }
  
private:
  std::ifstream input_file_;
  std::vector<std::pair<std::string, Vector7d>> q_data_;
};

void eigenToArray(const Vector7d & src, std::array<double, 7> & dst)
{
  for (int i=0; i<7; i++)
  {
    dst[i] = src(i);
  }
}

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  CSVJointPosReader cvs_joint_poses("calib_pos.cvs");
  cvs_joint_poses.print();
  
  try {
    franka::Robot robot(argv[1], franka::RealtimeConfig::kIgnore);
    setDefaultBehavior(robot);

    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    // First move the robot to a suitable joint configuration
    {
      std::array<double, 7> q_goal = {{0, 0, 0, -M_PI_2, 0, M_PI_2, M_PI_4}};
      MotionGenerator motion_generator(0.7, q_goal);
      std::cout << "WARNING: This example will move the robot! "
                << "Please make sure to have the user stop button at hand!" << std::endl
                << "Press Enter to continue..." << std::endl;
      std::cin.ignore();
      robot.control(motion_generator);
      std::cout << "Finished moving to initial joint configuration." << std::endl;
    }
    auto & q_data = cvs_joint_poses.getJointDatas();
    
    while(true)
    {
      std::cout << "Enter traj num: ";
      int num;  
      std::cin >> num;
      num --;
      if (num < 0) break;
      std::cout << "# Executing traj " << num+1 << std::endl <<
                   " - name: " << q_data[num].first << std::endl <<
                   " - qval: " << (q_data[num].second / M_PI * 180.0).transpose() << std::endl << std::endl;
      std::array<double, 7> q_goal;
      eigenToArray(q_data[num].second, q_goal);
      MotionGenerator motion_generator(0.7, q_goal);
      robot.control(motion_generator);
    }
    
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}