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
        # clean the scene
        scene.remove_world_object("table")
        scene.remove_world_object("cube")


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
        gripper = moveit_commander.MoveGroupCommander("panda_arm_hand")
        gripper.set_named_target("open")
        gripper.go()        

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
  main()
