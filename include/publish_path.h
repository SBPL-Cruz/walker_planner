#include "walker_planner/GraspPose.h"
#include "walker_planner/Path1.h"

void publish_path_fullbody(std::vector<smpl::RobotState> path, ros::Publisher path_pub)
{
  ROS_ERROR("publishing path");
  int number_of_joints = 16;
  ros::Rate r(10);
  // std::cerr<<"PATH SIXE "<<path.size()<<' '<<path[0].size();

  //while(ros::ok())
  for(int k=0; k<10000;k++)
  {
  walker_planner::Path1 Final_path;
  for(int i=0;i<path.size();i++)
    {
      // std::cerr<<i<<" "<<"path size "<<path.size()<<'\n';
      // std::cerr<<"states size "<<path[i].size()<<'\n';
      walker_planner::GraspPose State;
      for(int j=0;j<path[i].size();j++)
      {
          // std::cerr<<path[i][j]<<' ';
          State.a.push_back(path[i][j]);
      }
      // std::cerr<<'\n';
      Final_path.path.push_back(State);
    }
    path_pub.publish(Final_path);
  //ros::spinOnce();
  // r.sleep();
  }
}

void publish_path_base(std::vector<smpl::RobotState> path, ros::Publisher path_pub)
{
  ROS_ERROR("publishing path");
  int number_of_joints = 16;
  ros::Rate r(10);
  // std::cerr<<"PATH SIXE "<<path.size()<<' '<<path[0].size();

  //while(ros::ok())
  for(int k=0; k<10000;k++)
  {
  walker_planner::Path1 Final_path;
  for(int i=0;i<path.size();i++)
    {
      // std::cerr<<i<<" "<<"path size "<<path.size()<<'\n';
      // std::cerr<<"states size "<<path[i].size()<<'\n';
      walker_planner::GraspPose State;
      for(int j=0;j<path[i].size();j++)
      {
          // std::cerr<<path[i][j]<<' ';
          State.a.push_back(path[i][j]);
      }
      // std::cerr<<'\n';
      Final_path.path.push_back(State);
    }
    path_pub.publish(Final_path);
  //ros::spinOnce();
  // r.sleep();
  }
}


void publish_path(std::vector<smpl::RobotState> path, ros::Publisher path_pub)
{
  ROS_ERROR("publishing path");
  int number_of_joints = 16;
  ros::Rate r(10);
  // std::cerr<<"PATH SIXE "<<path.size()<<' '<<path[0].size();

  //while(ros::ok())
  for(int k=0; k<10000;k++)
  {
  walker_planner::Path1 Final_path;
  for(int i=0;i<path.size();i++)
    {
      // std::cerr<<i<<" "<<"path size "<<path.size()<<'\n';
      // std::cerr<<"states size "<<path[i].size()<<'\n';
      walker_planner::GraspPose State;
      for(int j=0;j<path[i].size();j++)
      {
          // std::cerr<<path[i][j]<<' ';
          State.a.push_back(path[i][j]);
      }
      // std::cerr<<'\n';
      Final_path.path.push_back(State);
    }
    path_pub.publish(Final_path);
  //ros::spinOnce();
  // r.sleep();
  }
}
