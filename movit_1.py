#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def main():   
    try:
        
        
        print("creating basic interface")
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_pick_demo', anonymous=True)

        #creating a robot object
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        ## This interface can be used to plan and execute motions:
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print "Groups available for planning:", robot.get_group_names()

        # State of the robot
        print "Printing robot state"
        print robot.get_current_state()
 
        #creating cube and table
        rospy.sleep(1)

        print"creating the scene"
        # publish a demo scene
        dem = moveit_commander.PoseStamped()
        dem.header.frame_id = robot.get_planning_frame()

        # add a pickup table
        dem.pose.position.x = 0.42
        dem.pose.position.y = -0.2
        dem.pose.position.z = 0.3
        scene.add_box("table1", dem, (0.1, 1.5, 0.6))

        # add a drop off table
        dem.pose.position.x = -0.42
        dem.pose.position.y = -0.2
        dem.pose.position.z = 0.3
        scene.add_box("table2", dem, (0.1, 1.5, 0.6))

        # add an object to be grasped
        dem.pose.position.x = 0.42
        dem.pose.position.y = -0.2
        dem.pose.position.z = 0.61
        scene.add_box("cube", dem, (0.035, 0.035, 0.035))
       

        #resetting the robot arm to prevent singularity
        print"moving joints to defined orientation to avoid singularity"
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -pi/4
        joint_goal[2] = 0
        joint_goal[3] = -pi/2
        joint_goal[4] = 0
        joint_goal[5] = pi/3
        joint_goal[6] = 0
         
        move_group.go(joint_goal, wait=True)
        move_group.stop()

        #opening the hand of the panda arm     
   
        #code to move the robot hand to the cube location
        
        #code to pick up the cube without collision
        
        #code to follow the problem statement trajectory
        
        #code to detach the cube from the hand
        

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
  main()
