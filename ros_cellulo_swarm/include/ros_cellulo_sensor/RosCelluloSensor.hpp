#pragma once
#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "ros_cellulo_swarm/ros_cellulo_sensor.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/transform_listener.h"


enum RelativePositions{upright,upleft,downleft,downright};

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class RosCelluloSensor
{
    public:
    /*!
     * Constructor.
     * @param nodeHandle the ROS node handle, MacAddr the MacAddress of this Cellulo robot
     */
    explicit RosCelluloSensor(ros::NodeHandle& nodeHandle);
    /*!
     * Destructor.
     */
    virtual ~RosCelluloSensor();

    private:

    /*!
     * Reads and verifies the ROS parameters.
     * @return true if successful.
     */
    bool readParameters();

    /*!
     * ROS topic callback method: Set threshold
     * @param std_msgs float64 for threshold
     */

    void topicCallback_setThreshold(const std_msgs::Float64 &velocity);



    //! ROS node handle.
    ros::NodeHandle& nodeHandle_;


    //! ROS topic subscriber.
    ros::Subscriber subscriber_setThreshold;

    //! ROS topic names to subscribe to.
    std::string subscriberTopic_setThreshold="/setThreshold";


    //! ROS topic publisher.
    ros::Publisher publisher_Obstacles, publisher_Robots;

    //! ROS topic name to publish to.
    std::string publisherTopic_Obstacles="detectedObstacles";
    std::string publisherTopic_Robots="detectedRobots";


    public:

    /*!
     * \brief looping to get the number of neighboring robots
     */
    bool getDetectedRobots();

    /*!
     * get distances to obstacles (including virtual walls(map edges)
     */
    bool getDetectedObstacles();

    double threshold;
    int my_number,nb_of_robots;
    double width_x,length_y;
    double scale;
    char **present_robots;
    ros_cellulo_swarm::ros_cellulo_sensor detected_robots_array;
    ros_cellulo_swarm::ros_cellulo_sensor detected_obstacles_array;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener *tf_listener;
    //tf::TransformListener tf_listener;
    //tf::StampedTransform transform;
    geometry_msgs::TransformStamped transform;
    geometry_msgs::TransformStamped mytransform;



};
