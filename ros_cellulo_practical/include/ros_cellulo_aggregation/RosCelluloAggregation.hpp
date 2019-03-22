#pragma once
#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "ros_cellulo_swarm/ros_cellulo_sensor.h"
#include "tf2_ros/transform_listener.h"

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class RosCelluloAggregation
{
    public:
    /*!
     * Constructor
     */
    explicit RosCelluloAggregation(ros::NodeHandle& nodeHandle);
    /*!
     * Destructor.
     */
    virtual ~RosCelluloAggregation();

    private:

    /*!
     * Reads and verifies the ROS parameters.
     * @return true if successful.
     */
    bool readParameters();


    //! ROS node handle.
    ros::NodeHandle& nodeHandle_;

    //! ROS topic subscribers.
    //TODO

    //! ROS topic names to subscribe to.
    // TODO

    /*!
     * \brief Topic callback functions
     */
    void topicCallback_getDetectedRobots(ros_cellulo_swarm::ros_cellulo_sensor sensor);
    void topicCallback_getDetectedObstacles(ros_cellulo_swarm::ros_cellulo_sensor sensor);
    void topicCallback_getVelocity(geometry_msgs::Vector3 velocity);


    public:

    /*!
     * setting the name of the publisher topics depending on the MacAddress of the cellulo
     */
    void setPublishers();

    /*!
     * setting the name of the subsciber topics depending on the MacAddress of the cellulo
     */
    void setSubscribers();
    void naive_calculate_new_velocities(); //  Part III step 1
    //void fourstates_calculate_new_velocities(); //  Part III step 2 Optional
    void field_calculate_new_velocities();  //  Part IV 
    
    geometry_msgs::Vector3 limit_velocity(geometry_msgs::Vector3 v,double limit);
    double norm(double x,double y);


    //Parameters
    double rate;
    double scale;
    double ko;
    char* mac_Adr;

    //! ROS publishers
    ros::Publisher publisher_Velocity;
    //! ROS subsribers
    ros::Subscriber subscriber_Robots;
    ros::Subscriber subscriber_Obstacles;
    ros::Subscriber subscriber_Velocity;

    /**/
    std::vector<geometry_msgs::Vector3> distances_to_robots;
    std::vector<geometry_msgs::Vector3> distances_to_obstacles;
    geometry_msgs::Vector3 velocity;
    int nb_of_robots_detected;
    int nb_of_obstacles_detected;
    bool velocity_updated;



};
