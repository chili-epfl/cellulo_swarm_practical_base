#include "ros_cellulo_reduced/RosCellulo.hpp"
#include "std_msgs/Time.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include <cstring>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <cstdio>


namespace Cellulo {

RosCellulo::RosCellulo(ros::NodeHandle& nodeHandle,char* MacAddr) : nodeHandle_(nodeHandle)
{

        if (!readParameters()) {

                ROS_ERROR("Could not read parameters :(.");
                ros::requestShutdown();
        }

        //subscribers
        subscriber_VisEff = nodeHandle_.subscribe(subscriberTopic_VisEff, 1, &RosCellulo::topicCallback_setVisualEffect, this);
        subscriber_setLEDResponseMode = nodeHandle_.subscribe(subscriberTopic_setLEDResponseMode, 1, &RosCellulo::topicCallback_setLEDResponseMode, this);
        subscriber_setGoalVelocity=nodeHandle_.subscribe(subscriberTopic_setGoalVelocity, 1, &RosCellulo::topicCallback_setGoalVelocity, this);
        subscriber_Reset = nodeHandle_.subscribe(subscriberTopic_Reset, 1, &RosCellulo::topicCallback_reset, this);
        subscriber_ClearTracking = nodeHandle_.subscribe(subscriberTopic_ClearTracking, 1, &RosCellulo::topicCallback_clearTracking, this);
        subscriber_shutDown= nodeHandle_.subscribe(subscriberTopic_shutDown, 1, &RosCellulo::topicCallback_shutDown, this);
        subscriber_ClearHapticFeedback = nodeHandle_.subscribe(subscriberTopic_ClearHapticFeedback, 1, &RosCellulo::topicCallback_clearHapticFeedback, this);
        subscriber_vibrateOnMotion = nodeHandle_.subscribe(subscriberTopic_vibrateOnMotion, 1, &RosCellulo::topicCallback_vibrateOnMotion, this);
        subscriber_simpleVibrate = nodeHandle_.subscribe(subscriberTopic_simpleVibrate, 1, &RosCellulo::topicCallback_simpleVibrate, this);
        subscriber_setGestureEnabled = nodeHandle_.subscribe(subscriberTopic_setGestureEnabled, 1, &RosCellulo::topicCallback_setGestureEnabled, this);
        subscriber_setCasualBackdriveAssistEnabled = nodeHandle_.subscribe(subscriberTopic_setCasualBackdriveAssistEnabled, 1, &RosCellulo::topicCallback_setCasualBackdriveAssistEnabled, this);


        //Publishers
        publisher_TouchKey = nodeHandle_.advertise<ros_cellulo_swarm::cellulo_touch_key>(publisherTopic_TouchKey, 10);
        publisher_LongTouchKey = nodeHandle_.advertise<ros_cellulo_swarm::cellulo_touch_key>(publisherTopic_LongTouchKey, 10);
        publisher_Kidnapped = nodeHandle_.advertise<ros_cellulo_swarm::cellulo_kidnapped_msg>(publisherTopic_Kidnapped, 10);
        publisher_Velocity=nodeHandle_.advertise<geometry_msgs::Vector3>(publisherTopic_Velocity,10);

        publisher_ConnectionStatus = nodeHandle_.advertise<std_msgs::Int8>(publisherTopic_ConnectStatus, 10);
        publisher_LocalAdapterMacAddress = nodeHandle_.advertise<std_msgs::String>(publisherTopic_LocalAdapterAdress, 10);
        publisher_autoConnect = nodeHandle_.advertise<std_msgs::Bool>(publisherTopic_autoConnect, 10);
        publisher_BatteryState = nodeHandle_.advertise<std_msgs::Int8>(publisherTopic_BatteryState, 10);
        publisher_Gesture = nodeHandle_.advertise<std_msgs::Int8>(publisherTopic_Gesture, 10);
        publisher_lastTimestamp = nodeHandle_.advertise<std_msgs::Int8>(publisherTopic_lastTimestamp, 10);
        publisher_framerate = nodeHandle_.advertise<std_msgs::Float32>(publisherTopic_framerate, 10);
        publisher_cameraImageProgress = nodeHandle_.advertise<std_msgs::Float32>(publisherTopic_cameraImageProgress, 10);
        publisher_poseVelControlEnabled = nodeHandle_.advertise<std_msgs::Bool>(publisherTopic_poseVelControlEnabled, 10);
        publisher_poseVelControlPeriod = nodeHandle_.advertise<std_msgs::Int32>(publisherTopic_poseVelControlPeriod, 10);
        publisher_marker_robot = nodeHandle_.advertise<visualization_msgs::Marker>(publisherTopic_marker_robot, 10);
        publisher_marker_traj = nodeHandle_.advertise<visualization_msgs::Marker>(publisherTopic_marker_traj, 10);


        //Services
        serviceServer_ = nodeHandle_.advertiseService("get_state",&RosCellulo::serviceCallback, this);

        //Link the object to the unique Mac Address
        RosCellulo::setMacAddr(MacAddr);
        RosCellulo::setAutoConnect(1);

        //RESET THE ROBOT
        RosCellulo::reset_counter=0;
        RosCellulo::reset();
        //RosCellulo::setPoseVelControlPeriod(25);

        //connect signals to publisher
        connect(this,SIGNAL(keysTouchedChanged()),SLOT(Publish_keys()));
        connect(this,SIGNAL(keysLongTouchedChanged()),SLOT(Publish_longKeys()));
        connect(this,SIGNAL(kidnappedChanged()),SLOT(Publish_Kidnapped()));
        connect(this,SIGNAL(connectionStatusChanged()),SLOT(Publish_connectionStatus()));
        connect(this,SIGNAL(autoConnectChanged()),SLOT(Publish_autoConnect()));
        connect(this,SIGNAL(macAddrChanged()),SLOT(Publish_macAddress()));
        connect(this,SIGNAL(batteryStateChanged()),SLOT(Publisher_batteryState()));
        connect(this,SIGNAL(gestureChanged()),SLOT(Publish_gesture()));
        connect(this,SIGNAL(timestampChanged()),SLOT(Publish_lastTimestamp()));
        connect(this,SIGNAL(timestampChanged()),SLOT(Publish_framerate()));
        connect(this,SIGNAL(cameraImageProgressChanged()),SLOT(Publish_cameraImageProgress()));
        connect(this,SIGNAL(poseChanged(qreal,qreal,qreal)),SLOT(Publish_pose()));
        connect(this,SIGNAL(vxywChanged()),SLOT(Publish_velocity()));
        connect(this,SIGNAL(poseVelControlEnabledChanged()),SLOT(Publish_poseVelControlEnable()));
        connect(this,SIGNAL(poseVelControlPeriodChanged()),SLOT(Publish_poseVelControlPeriod()));

}

RosCellulo::~RosCellulo(){
}

bool RosCellulo::readParameters()
{
        if (!nodeHandle_.getParam("scale_coord", scale)) return false;
        return true;
}

void RosCellulo::topicCallback_setVisualEffect(const ros_cellulo_swarm::cellulo_visual_effect& message)
{
        //ROS_INFO("Setting Visual Effect ");
        color.setBlue(message.blue);
        color.setGreen(message.green);
        color.setRed(message.red);
        Cellulo::CelluloBluetoothEnums::VisualEffect effect= static_cast<Cellulo::CelluloBluetoothEnums::VisualEffect>(message.effect);
        RosCellulo::setVisualEffect(effect,color,message.value);

}

void RosCellulo::topicCallback_setLEDResponseMode(const std_msgs::UInt8 mode)
{
        /** LEDs respond to touches by slightly increasing brightness    LEDResponseModeResponsiveIndividual = 0  */
        /** LEDs don't respond to touches
           LEDResponseModeAbsolute = 1  */
        /** LEDs respond to hold by all changing color
           LEDResponseModeResponsiveHold = 2  */

        Cellulo::CelluloBluetoothEnums::LEDResponseMode ledMode = static_cast<Cellulo::CelluloBluetoothEnums::LEDResponseMode>(mode.data);
        RosCellulo::setLEDResponseMode(ledMode);

}


void RosCellulo::topicCallback_setGoalVelocity(const geometry_msgs::Vector3 &velocity){
        RosCellulo::setGoalVelocity(velocity.x,velocity.y,velocity.z);
}


void RosCellulo::topicCallback_vibrateOnMotion(const ros_cellulo_swarm::cellulo_vibrateOnMotion& message )
{

        RosCellulo::vibrateOnMotion(message.iCoeff,message.period);

}


void RosCellulo::topicCallback_simpleVibrate(const ros_cellulo_swarm::cellulo_simpleVibrate& message )
{

        RosCellulo::simpleVibrate(message.pose.x,message.pose.y, message.pose.theta, message.period, message.duration);

}

void RosCellulo::topicCallback_setGestureEnabled(const std_msgs::Bool &message )
{

        RosCellulo::setGestureEnabled(message.data);

}

void RosCellulo::topicCallback_setCasualBackdriveAssistEnabled(const std_msgs::Bool &message )
{

        RosCellulo::setCasualBackdriveAssistEnabled(message.data);

}


void RosCellulo::Publish_pose()
{

        //ROS_INFO("broadcasting my pose.");
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "paper_world";
        std::string mac_adr_str=RosCellulo::getMacAddr().toStdString();
        transformStamped.child_frame_id = mac_adr_str.substr(0,2)+"_"+mac_adr_str.substr(3,2)+"_"+mac_adr_str.substr(6,2)+"_"+mac_adr_str.substr(9,2)+"_"+mac_adr_str.substr(12,2)+"_"+mac_adr_str.substr(15,2);
        transformStamped.transform.translation.x = RosCellulo::getX()*scale;
        transformStamped.transform.translation.y = RosCellulo::getY()*scale;
        transformStamped.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0,  RosCellulo::getTheta()/180*M_PI);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        br.sendTransform(transformStamped);

        /* Display a cube to visulize cellulo robot */
        visualization_msgs::Marker points;
        visualization_msgs::Marker robot;
        points.header.frame_id = robot.header.frame_id =transformStamped.child_frame_id;
        points.header.stamp = robot.header.stamp =ros::Time::now();
        points.action =robot.action=visualization_msgs::Marker::ADD;
        points.ns=robot.ns="ros_cellulo_swarm";
        points.pose.orientation.w = robot.pose.orientation.w=1.0;
        points.id = f;
        robot.id=0;
        points.type = visualization_msgs::Marker::SPHERE;
        robot.type = visualization_msgs::Marker::MESH_RESOURCE;

        // POINTS markers use x and y scale for width/height respectively
        robot.scale.x = scale;
        robot.scale.y = scale;
        robot.scale.z = scale;

        points.scale.x = 20*scale;
        points.scale.y = 20*scale;
        points.scale.z = 20*scale;

        // Points are green
        points.color.r = 1.0f;
        points.color.a = 1.0;
        points.lifetime=ros::Duration(20);

        robot.color.r = color.redF();
        robot.color.g = color.greenF();
        robot.color.b = color.blueF();
        robot.color.a = 1.0;
        robot.lifetime=ros::Duration(20);

        points.pose.position.x=0;
        points.pose.position.y=0;
        points.pose.position.z=0;

        robot.pose.position.x=0;
        robot.pose.position.y=0;
        robot.pose.position.z=0;

        f=f%1000+1; //display points of the trajectory up to 1000 points


        robot.mesh_resource = "package://ros_cellulo_swarm/meshes/cellulo.STL";
        //ROS_INFO("%d",f);
        publisher_marker_robot.publish(robot);
        publisher_marker_traj.publish(points);
}






bool RosCellulo::serviceCallback(ros_cellulo_swarm::CelluloState::Request& request,
                                 ros_cellulo_swarm::CelluloState::Response& response)
{

        ROS_INFO("Sending the robot state ");
        response.Position.header.stamp = ros::Time::now();
        response.Position.header.frame_id = "paper_world";
        std::string mac_adr_str=RosCellulo::getMacAddr().toStdString();
        response.Position.child_frame_id = mac_adr_str.substr(0,2)+"_"+mac_adr_str.substr(3,2)+"_"+mac_adr_str.substr(6,2)+"_"+mac_adr_str.substr(9,2)+"_"+mac_adr_str.substr(12,2)+"_"+mac_adr_str.substr(15,2);

        response.Position.transform.translation.x = RosCellulo::getX();
        response.Position.transform.translation.y = RosCellulo::getY();
        response.Position.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0,  RosCellulo::getTheta()/180*M_PI);
        response.Position.transform.rotation.x = q.x();
        response.Position.transform.rotation.y = q.y();
        response.Position.transform.rotation.z = q.z();
        response.Position.transform.rotation.w = q.w();

        response.vx.data=RosCellulo::getVx();
        response.vy.data=RosCellulo::getVy();
        response.w.data=RosCellulo::getW();

        response.kidnapped=RosCellulo::getKidnapped();

        int i;
        for (i=0; i<6; i++)
        {
                response.keysTouched.push_back(RosCellulo::getKeysTouched()[i]);
                response.keysLongTouched.push_back(RosCellulo::getKeysLongTouched()[i]);
        }

        return true;
}

void RosCellulo::Publish_keys()
{

        ros_cellulo_swarm::cellulo_touch_key keys;
        int i;
        for (i=0; i<6; i++)
        {
                keys.keysTouched.push_back(RosCellulo::getKeysTouched()[i]);
        }
        std::string mac_adr_str=RosCellulo::getMacAddr().toStdString();
        keys.header.frame_id=mac_adr_str.substr(0,2)+"_"+mac_adr_str.substr(3,2)+"_"+mac_adr_str.substr(6,2)+"_"+mac_adr_str.substr(9,2)+"_"+mac_adr_str.substr(12,2)+"_"+mac_adr_str.substr(15,2);
        keys.header.stamp= ros::Time::now();
        publisher_TouchKey.publish(keys);
}

void RosCellulo::Publish_longKeys()
{

        ros_cellulo_swarm::cellulo_touch_key keys; //= new cellulo_touch_key[6];
        int i;
        for (i=0; i<6; i++)
        {
                keys.keysTouched.push_back(RosCellulo::getKeysLongTouched()[i]);
        }
        std::string mac_adr_str=RosCellulo::getMacAddr().toStdString();
        keys.header.frame_id=mac_adr_str.substr(0,2)+"_"+mac_adr_str.substr(3,2)+"_"+mac_adr_str.substr(6,2)+"_"+mac_adr_str.substr(9,2)+"_"+mac_adr_str.substr(12,2)+"_"+mac_adr_str.substr(15,2);
        keys.header.stamp= ros::Time::now();
        publisher_LongTouchKey.publish(keys);
}

void RosCellulo::Publish_kidnapped(){

        ros_cellulo_swarm::cellulo_kidnapped_msg kidnapped;
        kidnapped.Kidnapped=RosCellulo::getKidnapped();
        std::string mac_adr_str=RosCellulo::getMacAddr().toStdString();
        kidnapped.header.frame_id=mac_adr_str.substr(0,2)+"_"+mac_adr_str.substr(3,2)+"_"+mac_adr_str.substr(6,2)+"_"+mac_adr_str.substr(9,2)+"_"+mac_adr_str.substr(12,2)+"_"+mac_adr_str.substr(15,2);
        kidnapped.header.stamp= ros::Time::now();
        publisher_Kidnapped.publish(kidnapped);
}

void RosCellulo::Publish_connectionStatus(){

        std_msgs::Int8 connecSt;
        connecSt.data=RosCellulo::getConnectionStatus();
        publisher_ConnectionStatus.publish(connecSt);
}
void RosCellulo::Publish_localAdapterMacAddress(){

        std_msgs::String LocalAdapMacAd;
        LocalAdapMacAd.data= RosCellulo::getLocalAdapterMacAddr().toStdString();
        publisher_LocalAdapterMacAddress.publish(LocalAdapMacAd);
}

void RosCellulo::Publish_autoConnect(){

        std_msgs::Bool autoCon;
        autoCon.data=RosCellulo::getAutoConnect();
        publisher_autoConnect.publish(autoCon);
}

void RosCellulo::Publish_macAddress(){

        std_msgs::String macAd;
        macAd.data=RosCellulo::getMacAddr().toStdString();
        publisher_macAddress.publish(macAd);
        ROS_INFO("mac address is %s \n", macAd.data.c_str());
}

void RosCellulo::Publisher_batteryState(){

        std_msgs::Int8 BatState;
        BatState.data=RosCellulo::getBatteryState();
        publisher_BatteryState.publish(BatState);
}

void RosCellulo::Publish_gesture(){

        std_msgs::Int8 gest;
        gest.data=RosCellulo::getGesture();
        publisher_Gesture.publish(gest);
}


void RosCellulo::Publish_lastTimestamp(){

        std_msgs::Int8 lastTimeStamp;
        lastTimeStamp.data=RosCellulo::getLastTimestamp();


        publisher_lastTimestamp.publish(lastTimeStamp);
}

void RosCellulo::Publish_framerate(){

        std_msgs::Float32 frameRate;
        frameRate.data=RosCellulo::getFramerate();
        publisher_framerate.publish(frameRate);
}

void RosCellulo::Publish_cameraImageProgress(){
        std_msgs::Float32 CameraImageProgress;
        CameraImageProgress.data=RosCellulo::getCameraImageProgress();

        publisher_cameraImageProgress.publish(CameraImageProgress);
}

void RosCellulo::Publish_velocity(){

    geometry_msgs::Vector3 v;
    v.x=RosCellulo::getVx()*scale;
    v.y=RosCellulo::getVy()*scale;
    v.z=RosCellulo::getW()*scale;
    //ROS_INFO("Getting velocity: %lf,%lf,%lf",v.x,v.y,v.z);

    publisher_Velocity.publish(v);
}



void RosCellulo::Publish_poseVelControlEnable(){
        std_msgs::Bool e;
        e.data=RosCellulo::getPoseVelControlEnabled();

        publisher_poseVelControlEnabled.publish(e);
}

void RosCellulo::Publish_poseVelControlPeriod(){
        std_msgs::Int32 p;
        p.data=RosCellulo::getPoseVelControlPeriod();

        publisher_poseVelControlPeriod.publish(p);
}
void RosCellulo::topicCallback_clearTracking(const std_msgs::Empty message){
        RosCellulo::clearTracking();
}

void RosCellulo::topicCallback_clearHapticFeedback(const std_msgs::Empty message){
        RosCellulo::clearHapticFeedback();
}

void RosCellulo::topicCallback_reset(const std_msgs::Empty message)
{
        RosCellulo::reset_counter=0;
        RosCellulo::reset_robot();

}

bool RosCellulo::reset_robot(){
        if(RosCellulo::getConnectionStatus()==Cellulo::CelluloBluetoothEnums::ConnectionStatusConnected)
        {
                RosCellulo::reset();
                ROS_INFO("resetting");
                return true;
        }
        return false;
}

void RosCellulo::topicCallback_shutDown(const std_msgs::Empty &empt)
{

        RosCellulo::shutdown();
}


double RosCellulo::norm(double x,double y)
{
        return sqrt(pow(x,2)+pow(y,2));
}

} /* namespace */
