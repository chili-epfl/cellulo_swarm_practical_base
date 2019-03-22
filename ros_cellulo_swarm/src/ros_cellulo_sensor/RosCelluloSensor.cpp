#include "ros_cellulo_sensor/RosCelluloSensor.hpp"
#include "tf2_ros/transform_listener.h"

RosCelluloSensor::RosCelluloSensor(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle)
{
        if (!readParameters()) {

                ROS_ERROR("Could not read parameters :(.");
                ros::requestShutdown();
        }
        //subscribers
        subscriber_setThreshold = nodeHandle_.subscribe(subscriberTopic_setThreshold, 1, &RosCelluloSensor::topicCallback_setThreshold, this);

        //listener
        tf_listener= new tf2_ros::TransformListener(tfBuffer);

        //publishers
        publisher_Robots =nodeHandle_.advertise<ros_cellulo_swarm::ros_cellulo_sensor>(publisherTopic_Robots,1);
        publisher_Obstacles = nodeHandle_.advertise<ros_cellulo_swarm::ros_cellulo_sensor>(publisherTopic_Obstacles,1);

}

RosCelluloSensor::~RosCelluloSensor(){
}

bool RosCelluloSensor::readParameters()
{
        if (!nodeHandle_.getParam("threshold", threshold))
            return false;
        else if(!nodeHandle_.getParam("cellulo_num", my_number))
            return false;
        else if(!nodeHandle_.getParam("paper_width", width_x))
            return false;
        else if(!nodeHandle_.getParam("paper_length", length_y))
            return false;
        else if(!nodeHandle_.getParam("scale", scale))
            return false;

        return true;
}

void RosCelluloSensor::topicCallback_setThreshold(const std_msgs::Float64& message)
{
        threshold=message.data;
}


bool RosCelluloSensor::getDetectedRobots()
{
    double now=ros::Time::now().toSec();
    char* my_frame=present_robots[my_number];
    bool return_value=false;

    detected_robots_array.Distance.clear();
    detected_robots_array.detected=0;

    double distance;
    uint8_t cellulo_nb_in_field=0;

    try{

        mytransform=tfBuffer.lookupTransform("paper_world",my_frame,ros::Time(0));//::now(),ros::Duration(0.03));
    }
    catch(tf2::TransformException &ex){
        //ROS_WARN("%s",ex.what());
        return false;
    }




    if((now-mytransform.header.stamp.toSec())>1)
        return false;
    //ROS_INFO("node %d, time diff: %lf",my_number,now-mytransform.header.stamp.toSec());

    double my_position_x=mytransform.transform.translation.x;
    double my_position_y=mytransform.transform.translation.y;

    for(int i=1;i<=nb_of_robots ;i++)
    {
        if(i==my_number)
            continue;

        char* other_frame=present_robots[i];
        try{
            transform=tfBuffer.lookupTransform("paper_world",other_frame,ros::Time(0));//::now(),ros::Duration(0.03));
        }
        catch(tf2::TransformException &ex){
            //ROS_WARN("%s",ex.what());
            continue;
        }
        if((now-transform.header.stamp.toSec())>1)
            continue;

        distance=sqrt(pow(transform.transform.translation.x-my_position_x,2)+pow(transform.transform.translation.y-my_position_y,2));

        if(distance<threshold)
        {
            return_value=true;
            geometry_msgs::Vector3 d;
            d.x=transform.transform.translation.x-my_position_x;
            d.y=transform.transform.translation.y-my_position_y;
            d.z=distance;
            detected_robots_array.Distance.push_back(d);
            cellulo_nb_in_field++;
        }
        detected_robots_array.detected=cellulo_nb_in_field;

    }

    detected_robots_array.timestamp=ros::Time::now();
    publisher_Robots.publish(detected_robots_array);
    return return_value;
}

bool RosCelluloSensor::getDetectedObstacles()
{
    if(ros::Time::now().toSec()-mytransform.header.stamp.toSec()>1)
        return false;

    bool return_value=false;

    detected_obstacles_array.Distance.clear();
    detected_robots_array.detected=0;

    double distance[4],x[4],y[4];

    uint8_t obstacles_nb_in_field=0;

    //distances to walls.
    x[0]=0;
    y[0]=-mytransform.transform.translation.y;
    distance[0]=fabs(mytransform.transform.translation.y);

    x[1]=(width_x)-mytransform.transform.translation.x;
    y[1]=0;
    distance[1]=(width_x)-mytransform.transform.translation.x;

    x[2]=0;
    y[2]=(length_y)-mytransform.transform.translation.y;
    distance[2]=(length_y)-mytransform.transform.translation.y;

    x[3]=-mytransform.transform.translation.x;
    y[3]=0;
    distance[3]=fabs(mytransform.transform.translation.x);

    for(int i=0;i<4;i++)
    {
        if(distance[i]<threshold)
        {
            return_value=true;
            geometry_msgs::Vector3 d;
            d.x=x[i];
            d.y=y[i];
            d.z=distance[i];
            detected_obstacles_array.Distance.push_back(d);
            obstacles_nb_in_field++;
        }
        detected_obstacles_array.detected=obstacles_nb_in_field;

    }

    publisher_Obstacles.publish(detected_obstacles_array);
    return return_value;
}

