#pragma once

#include "CelluloRobot.h"
// ROS
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "std_msgs/UInt8.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"

#include "ros_cellulo_swarm/cellulo_visual_effect.h"
#include "ros_cellulo_swarm/CelluloState.h"
#include "ros_cellulo_swarm/cellulo_kidnapped_msg.h"
#include "ros_cellulo_swarm/cellulo_touch_key.h"
#include "ros_cellulo_swarm/cellulo_vibrateOnMotion.h"
#include "ros_cellulo_swarm/cellulo_coord.h"
#include "ros_cellulo_swarm/cellulo_simpleVibrate.h"
#include "ros_cellulo_swarm/cellulo_pose_velocity.h"



//QT
#include "QObject"

namespace Cellulo {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class RosCellulo : public CelluloRobot
{
Q_OBJECT

public:
/*!
 * Constructor.
 * @param nodeHandle the ROS node handle, MacAddr the MacAddress of this Cellulo robot
 */
explicit RosCellulo(ros::NodeHandle& nodeHandle,char* MacAddr);


/*!
 * Destructor.
 */
virtual ~RosCellulo();

private:

/*!
 * Reads and verifies the ROS parameters.
 * @return true if successful.
 */
bool readParameters();


/*!
 * ROS topic callback method: Set thevelocity
 * @param ros_cellulo_abstract_msgs of velocity
 */

void topicCallback_setGoalVelocity(const geometry_msgs::Vector3 &velocity);

/*!
 * ROS topic callback method: Set the visual effect.
 * @param ros_cellulo_msgs of visual effect
 */

void topicCallback_setVisualEffect(const ros_cellulo_swarm::cellulo_visual_effect& message);

/*!
 * ROS topic callback method Sets the LED response mode, i.e the LED visual response of the robot to touches
 * @param uint8 LED resposne mode
 */
void topicCallback_setLEDResponseMode(const std_msgs::UInt8 mode);


/*!
 * ROS topic callback method: Resets the Cellulo robot
 * @param empty message
 */

void topicCallback_reset(const std_msgs::Empty message);

/*!
 * ROS topic callback method.
 * Shutdown cellulo
 */

void topicCallback_shutDown(const std_msgs::Empty &empt);

/*!
 * ROS topic callback method
 * Clear Tracking of the robot.
 */

void topicCallback_clearTracking(const std_msgs::Empty message);

/*!
 * ROS topic callback method
 * Clear Haptic Feedback of the robot.
 */

void topicCallback_clearHapticFeedback(const std_msgs::Empty message);


/**
 * @brief Constantly vibrates the robot
 *
 * @param iX X intensity, scale is the same as linear velocity
 * @param iY Y intensity, scale is the same as linear velocity
 * @param iTheta Theta intensity, scale is the same as angular velocity
 * @param period Period of vibration in milliseconds, maximum is 0xFFFF
 * @param duration Duration of vibration in milliseconds, maximum is 0xFFFF, 0 for vibrate forever
 */
void topicCallback_simpleVibrate(const ros_cellulo_swarm::cellulo_simpleVibrate &message);

/*!
 * ROS topic callback method
 * Simple Vibrate
 */

void topicCallback_vibrateOnMotion(const ros_cellulo_swarm::cellulo_vibrateOnMotion &message);


/*!
 * ROS topic callback method
 * setGestureEnabled.
 */

void topicCallback_setGestureEnabled(const std_msgs::Bool &message);

/*!
 * ROS topic callback method
 * setCasualBackdriveAssistEnabled
 */
void topicCallback_setCasualBackdriveAssistEnabled(const std_msgs::Bool &message );

/*!
 * ROS service server callback.
 * @param request the request of the service.
 * @param response the provided response.
 * @return true if successful, false otherwise.
 */
bool serviceCallback(ros_cellulo_swarm::CelluloState::Request& request,
                     ros_cellulo_swarm::CelluloState::Response& response);



//! ROS node handle.
ros::NodeHandle& nodeHandle_;


//! ROS topic subscriber.
ros::Subscriber subscriber_VisEff;
ros::Subscriber subscriber_setLEDResponseMode;
ros::Subscriber subscriber_setGoalVelocity;
ros::Subscriber subscriber_Reset;
ros::Subscriber subscriber_shutDown;
ros::Subscriber subscriber_ClearTracking;
ros::Subscriber subscriber_ClearHapticFeedback;
ros::Subscriber subscriber_vibrateOnMotion;
ros::Subscriber subscriber_simpleVibrate;
ros::Subscriber subscriber_setGestureEnabled;
ros::Subscriber subscriber_setCasualBackdriveAssistEnabled;


//! ROS topic names to subscribe to.
std::string subscriberTopic_setGoalVelocity="setGoalVelocity";
std::string subscriberTopic_VisEff="setVisualEffect";
std::string subscriberTopic_setLEDResponseMode="setLEDResponseMode";
std::string subscriberTopic_Reset="reset";
std::string subscriberTopic_shutDown="shutdown";
std::string subscriberTopic_ClearTracking="clearTracking";
std::string subscriberTopic_ClearHapticFeedback="clearHapticFeedback";
std::string subscriberTopic_vibrateOnMotion="vibrateOnMotion";
std::string subscriberTopic_simpleVibrate="simpleVibrate";
std::string subscriberTopic_setGestureEnabled="setGestureEnabled";
std::string subscriberTopic_setCasualBackdriveAssistEnabled="setCasualBackdriveAssistEnabled";

//! ROS topic publisher.
ros::Publisher publisher_TouchKey;
ros::Publisher publisher_LongTouchKey;
ros::Publisher publisher_Kidnapped;
ros::Publisher publisher_Velocity;
ros::Publisher publisher_ConnectionStatus;
ros::Publisher publisher_LocalAdapterMacAddress;
ros::Publisher publisher_autoConnect;
ros::Publisher publisher_macAddress;
ros::Publisher publisher_BatteryState;
ros::Publisher publisher_Gesture;
ros::Publisher publisher_lastTimestamp;
ros::Publisher publisher_framerate;
ros::Publisher publisher_cameraImageProgress;
ros::Publisher publisher_poseVelControlEnabled;
ros::Publisher publisher_poseVelControlPeriod;
ros::Publisher publisher_marker_robot;
ros::Publisher publisher_marker_traj;




//! ROS topic name to publish to.
std::string publisherTopic_TouchKey="touchKey";
std::string publisherTopic_LongTouchKey="longTouchKey";
std::string publisherTopic_Kidnapped="kidnapped";
std::string publisherTopic_ConnectStatus="connectionStatus";
std::string publisherTopic_LocalAdapterAdress="localAdapterAdress";
std::string publisherTopic_autoConnect="autoConnect";
std::string publisherTopic_macAddress="macAddress";
std::string publisherTopic_BatteryState="batteryState";
std::string publisherTopic_Gesture="gesture";
std::string publisherTopic_lastTimestamp="lastTimestamp";
std::string publisherTopic_framerate="framerate";
std::string publisherTopic_cameraImageProgress="cameraImageProgress";
std::string publisherTopic_poseVelControlEnabled="poseVelControlEnable";
std::string publisherTopic_poseVelControlPeriod="poseVelControlPeriod";
std::string publisherTopic_marker_robot="visualization_marker_robot";
std::string publisherTopic_marker_traj="visualization_marker_traj";
std::string publisherTopic_Velocity="velocity";

//! ROS service server.
ros::ServiceServer serviceServer_;

int f=0;         // counter for the ids for the markers of the trajectory of cellulo (in rviz)
double scale=1;

QColor color = QColor("white");         // color of Cellulo leds

//function to calculate the norm of a vector (x,y)
double norm(double x,double y);

//Publishers defined as slots

public slots:
void Publish_pose();
void Publish_longKeys();
void Publish_keys();
void Publish_kidnapped();
void Publish_connectionStatus();
void Publish_localAdapterMacAddress();
void Publish_autoConnect();
void Publish_macAddress();
void Publisher_batteryState();
void Publish_gesture();
void Publish_lastTimestamp();
void Publish_framerate();
void Publish_cameraImageProgress();
void Publish_velocity();
void Publish_poseVelControlEnable();
void Publish_poseVelControlPeriod();
bool reset_robot();

public:
int reset_counter=0;         //reset_counter to resend the reset command

};
} /* namespace */
