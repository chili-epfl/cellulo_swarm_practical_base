#!/usr/bin/env python

from __future__ import print_function

import roslib#; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist,Vector3

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   q    w    e
   a    .    d
   z    s    x
   
i/k : increase/decrease max speeds by 20%
anything else : stop
CTRL-C to quit
"""

moveBindings = {
        's':(1,0,0,0),
        'a':(0,1,0,0),
        'd':(0,-1,0,0),
        'w':(-1,0,0,0),
        'q':(-1,1,0,0),
        'e':(-1,-1,0,0),
        'z':(1,1,0,0),
        'x':(1,-1,0,0)
    }

speedBindings={
        'i':(1.2,1.2),
        'k':(0.8,.8)
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 20)
    turn = rospy.get_param("~turn", 1)
    robot=rospy.get_param('~macAddr', '1')
    print(robot)
    pub = rospy.Publisher('/cellulo_node_'+robot+'/setGoalVelocity', Vector3, queue_size = 1)
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
            pub.publish(twist.linear)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist.linear)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)