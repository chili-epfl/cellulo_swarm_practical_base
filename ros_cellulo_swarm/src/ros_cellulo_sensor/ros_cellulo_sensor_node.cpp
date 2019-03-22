#include <ros/ros.h>
#include "ros_cellulo_sensor/RosCelluloSensor.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_cellulo_sensor");
    ros::NodeHandle nodeHandle("~");
    RosCelluloSensor rosCelluloSensor(nodeHandle);

    rosCelluloSensor.present_robots=argv;
    rosCelluloSensor.nb_of_robots=argc-1;

    double r;
    r=50;
    ros::Rate rate(r);
    
    while(ros::ok())
    {
        rosCelluloSensor.getDetectedRobots();
        rosCelluloSensor.getDetectedObstacles();
        ros::spinOnce();
        rate.sleep();
    }

  return 0;
}
