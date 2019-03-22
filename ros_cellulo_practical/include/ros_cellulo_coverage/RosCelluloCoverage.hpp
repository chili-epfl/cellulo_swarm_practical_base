#pragma once
#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "ros_cellulo_swarm/ros_cellulo_sensor.h"
#include "tf2_ros/transform_listener.h"

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class RosCelluloCoverage
{
    public:
    /*!
     * Constructor
     */
    explicit RosCelluloCoverage(ros::NodeHandle& nodeHandle);
    /*!
     * Destructor.
     */
    virtual ~RosCelluloCoverage();

    private:

    /*!
     * Reads and verifies the ROS parameters.
     * @return true if successful.
     */
    bool readParameters();

    /*!
     * ROS topic callback methods: Set threshold
     */

    void topicCallback_setFieldStrengthObstacles(const std_msgs::Float64 &k);
    void topicCallback_setFieldStrengthRobots(const std_msgs::Float64 &k);
    void topicCallback_setMass(const std_msgs::Float64 &m);
    void topicCallback_setViscocity(const std_msgs::Float64 &mu);

    //! ROS node handle.
    ros::NodeHandle& nodeHandle_;

    //! ROS topic subscribers.
    ros::Subscriber subscriber_setFieldStrengthObstacles;
    ros::Subscriber subscriber_setFieldStrengthRobots;
    ros::Subscriber subscriber_setMass;
    ros::Subscriber subscriber_setViscocity;

    //! ROS topic names to subscribe to.
    std::string subscriberTopic_setFieldStrengthObstacles="/setFieldStrengthObstacles";
    std::string subscriberTopic_setFieldStrengthRobots="/setFieldStrengthRobots";
    std::string subscriberTopic_setMass="/setMass";
    std::string subscriberTopic_setViscocity="/setViscocity";

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
    void calculate_new_velocities();
    
    geometry_msgs::Vector3 limit_velocity(geometry_msgs::Vector3 v,double limit);
    double norm(double x,double y);


    //Parameters
    double ko;
    double kr;
    double m;
    double mu;
    double rate;
    double scale;

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
