#include <ros/ros.h>
#include "ros_cellulo_reduced/RosCellulo.hpp"
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#define max_reset 3

int main(int argc, char** argv)
{

  /* get name from argv and transform it from xx_xx to xx:xx*/
  char* name=argv[1];
  name[2]=':';
  name[5]=':';
  name[8]=':';
  name[11]=':';
  name[14]=':';
  char MacAdr[17];
  sprintf (MacAdr, "%s",name);

  ROS_INFO("arg is %s \n",MacAdr);

  QCoreApplication app(argc, argv);

  ros::init(argc, argv, "ros_cellulo");
  ros::NodeHandle nodeHandle("~");

  Cellulo::RosCellulo rosCellulo(nodeHandle,MacAdr);

  ros::Rate rate(1000);

  while(ros::ok()){
      // when resetting needs to send the commands several times to guarantee reset.
      if(rosCellulo.reset_counter<max_reset)
      {
          if(rosCellulo.reset_robot())
              rosCellulo.reset_counter++;
      }
  app.processEvents();
  ros::spinOnce();
  rate.sleep();
  }

  return 0;
}
