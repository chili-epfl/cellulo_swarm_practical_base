#include <ros/ros.h>
#include "ros_cellulo_swarm/cellulo_touch_key.h"
#include "ros_cellulo_swarm/cellulo_visual_effect.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"

class LeaderSelection
{
public:
    explicit LeaderSelection(ros::NodeHandle& nodeHandle): nodeHandle_(nodeHandle){}
    
    //Parameters
    std_msgs::String leader; 
    char** present_robots; //points to the mac addresses of the robots present in the launch file. 
                           //Indexing starts at one so present_robots[1] is the the first robot, present_robots[2] the second, etc ...

    int nb_robots; // indicates the number of robots

    //! ROS node handle.
    ros::NodeHandle& nodeHandle_;

    //! ROS subscirbers and publishers.
    ros::Subscriber* TouchedRobots; //Subscribes to long touch events 
    ros::Publisher* VisualEffectPublisher; //Publishes visual effects (leds)
    ros::Publisher LeaderPublisher; //Publishes how is the leader. 

    void setSubscribersPublishers()
    {
        TouchedRobots=new ros::Subscriber[nb_robots];
        VisualEffectPublisher=new ros::Publisher[nb_robots];

        for(int i=0; i<nb_robots;i++)
        {
            char subscriberTopic[100];
            sprintf(subscriberTopic, "/cellulo_node_%s/longTouchKey",present_robots[i+1]);
            TouchedRobots[i]=nodeHandle_.subscribe(subscriberTopic, 10,&LeaderSelection::topicCallback_getTouchKeys,this);

            //Publishers
            char publisherTopic[100];
            sprintf(publisherTopic, "/cellulo_node_%s/setVisualEffect",present_robots[i+1]);
            VisualEffectPublisher[i] = nodeHandle_.advertise<geometry_msgs::Vector3>(publisherTopic,10);
        }
        LeaderPublisher=nodeHandle_.advertise<std_msgs::String>("/leader",1);
    }

    // Call back function if a change on one of the long touch sensors on one of the robots is detected. 
    // The functions should detects which robot was touched and publish its mac_adress on the LeaderPublisher
    void topicCallback_getTouchKeys(const std_msgs::String &msg)//ros_cellulo_swarm::cellulo_touch_key& message)
    {

        // a- detect which robot was selected.
        //    i- publish its mac address
        //   ii- turn its leds to red. 
        // b- turn the leds of other robots to green 
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_cellulo_leader");
    ros::NodeHandle nodeHandle("~");

    LeaderSelection leaderSelection(nodeHandle);

    leaderSelection.present_robots=argv;
    leaderSelection.nb_robots=argc-1;

    leaderSelection.setSubscribersPublishers();
    ros::spin();

    return 0;
}
