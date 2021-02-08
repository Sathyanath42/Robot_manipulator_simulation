#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_msgs.msg import Grasp
from moveit_msgs.msg import PlaceLocation
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler


def main():   
    try:
               
        print("creating basic interface")
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_and_pick', anonymous=True)

        #creating a robot object
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        #Defining the group name
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        move_group.set_planning_time(45.0)

        print "============ Printing initial robot state"
        print robot.get_current_state()

        ################################################### Creating the environment: Namely a cube and 2 tables to pick from and place on.
        #clean the scene
        scene.remove_world_object("table1")
        scene.remove_world_object("table2")
        scene.remove_world_object("cube")

        print"################Creating the scene"
        dem = moveit_commander.PoseStamped()
        dem.header.frame_id = "panda_link0"

        #add a pickup table
        dem.pose.position.x = 0.5
        dem.pose.position.y = 0
        dem.pose.position.z = 0.2
        scene.add_box("table1", dem, (0.02, 0.02, 0.58))

        #add a drop off table
        dem.pose.position.x = 0
        dem.pose.position.y = 0.5
        dem.pose.position.z = 0.2
        scene.add_box("table2", dem, (0.02, 0.02, 0.58))

        #add an object to be grasped
        dem.pose.position.x = 0.5
        dem.pose.position.y = 0
        dem.pose.position.z = 0.5
        scene.add_box("cube", dem, (0.02, 0.02, 0.02))
        rospy.sleep(1)

        ########################Resetting the robot arm to prevent singularity
        print"Resetting: moving joints to defined orientation to avoid singularity"
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
        rospy.sleep(1)

        ##################################################Approaching the cube and picking it up
        print "################Moving to pick the cube"
        grasps = Grasp()

        #defining the grasp pose of the end effector
        grasps.grasp_pose.header.frame_id = "panda_link0"
        grasps.grasp_pose.pose.position.x = 0.415
        grasps.grasp_pose.pose.position.y = 0
        grasps.grasp_pose.pose.position.z = 0.5
        quaternion = quaternion_from_euler(-pi/2, -pi/4, -pi/2)
        grasps.grasp_pose.pose.orientation.x = quaternion[0]
        grasps.grasp_pose.pose.orientation.y = quaternion[1]
        grasps.grasp_pose.pose.orientation.z = quaternion[2]
        grasps.grasp_pose.pose.orientation.w = quaternion[3]

        #setting the pregrasp approach
        grasps.pre_grasp_approach.direction.header.frame_id = "panda_link0"
        grasps.pre_grasp_approach.direction.vector.x = 1.0
        grasps.pre_grasp_approach.min_distance = 0.095
        grasps.pre_grasp_approach.desired_distance = 0.115
    
        #setting post grasp-retreat
        grasps.post_grasp_retreat.direction.header.frame_id = "panda_link0"
        grasps.post_grasp_retreat.direction.vector.z = 1.0
        grasps.post_grasp_retreat.min_distance = 0.1
        grasps.post_grasp_retreat.desired_distance = 0.25

        #opening the hand
        #add points
        grasps.pre_grasp_posture.joint_names.append("panda_finger_joint1")
        grasps.pre_grasp_posture.joint_names.append("panda_finger_joint2")
        point1=JointTrajectoryPoint()
        point1.positions.append(0.04)
        point1.positions.append(0.04)
        point1.time_from_start = rospy.Duration(0.5)
        grasps.pre_grasp_posture.points.append(point1)
     
        #closing the hand
        #add points
        grasps.grasp_posture.joint_names.append("panda_finger_joint1")
        grasps.grasp_posture.joint_names.append("panda_finger_joint2")
        point2=JointTrajectoryPoint()
        point2.positions.append(0.00)
        point2.positions.append(0.00)
        point2.time_from_start = rospy.Duration(0.5)
        grasps.grasp_posture.points.append(point2)

        move_group.set_support_surface_name("table1")
        move_group.pick("cube", grasps)

        rospy.sleep(1.0)

        print "============ Printing picked up robot state"
        print robot.get_current_state()

        ##########################################################Moving the cube and placing it down
        print"################Moving to place the cube"
        placer = PlaceLocation()
        #defining the grasp pose of the end effector
        placer.place_pose.header.frame_id = "panda_link0"
        placer.place_pose.pose.position.x = 0
        placer.place_pose.pose.position.y = 0.5
        placer.place_pose.pose.position.z = 0.5
        quaternion = quaternion_from_euler(-pi, 0, pi/2)
        placer.place_pose.pose.orientation.x = quaternion[0]
        placer.place_pose.pose.orientation.y = quaternion[1]
        placer.place_pose.pose.orientation.z = quaternion[2]
        placer.place_pose.pose.orientation.w = quaternion[3]

        #setting the pre-place approach
        placer.pre_place_approach.direction.header.frame_id = "panda_link0"
        placer.pre_place_approach.direction.vector.z = -1.0
        placer.pre_place_approach.min_distance = 0.095
        placer.pre_place_approach.desired_distance = 0.115
    
        #setting post place-retreat
        placer.post_place_retreat.direction.header.frame_id = "panda_link0"
        placer.post_place_retreat.direction.vector.y = -1.0
        placer.post_place_retreat.min_distance = 0.1
        placer.post_place_retreat.desired_distance = 0.25

        #opening the hand
        #add points
        placer.post_place_posture.joint_names.append("panda_finger_joint1")
        placer.post_place_posture.joint_names.append("panda_finger_joint2")
        point3 = JointTrajectoryPoint()
        point3.positions.append(0.00)
        point3.positions.append(0.00)
        point3.time_from_start = rospy.Duration(0.5)
        placer.post_place_posture.points.append(point3)

        move_group.set_support_surface_name("table2")
        move_group.place("cube", placer)

        print "============ Printing placed/final robot state"
        print robot.get_current_state()
       
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':   
    main()
  
