#!/usr/bin/env python
"""
===================================================
rosActuator.py - Actuation Halder for ROS intefarce
===================================================

Control functions using ROS
"""

import time
import threading, thread, subprocess
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('pr2_controllers_msgs')
roslib.load_manifest('pr2_tuckarm')

import rospy
import actionlib
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import *
from pr2_tuck_arms_action import tuck_arms_main

roslib.load_manifest('pr2_pick_and_place_demos')
from pr2_pick_and_place_demos.pick_and_place_demo import *
from pr2_controllers_msgs.msg import SingleJointPositionAction, SingleJointPositionGoal

#import signal
#roslib.load_manifest('pr2_tuck_arms_action')

#import os
#import sys
#import math

#from pr2_common_action_msgs.msg import *
#import getopt



import lib.handlers.handlerTemplates as handlerTemplates

class RosActuatorHandler(handlerTemplates.ActuatorHandler):
    def __init__(self, executor, shared_data):
        """
        Actuator Handler for ROS type applications
        """
        self.rosInitHandler = shared_data['ROS_INIT_HANDLER']
        self.loco = executor.hsub.getHandlerInstanceByType(handlerTemplates.LocomotionCommandHandler)

    #####################################
    ### Available actuator functions: ###
    #####################################


    def _actionTemplate(self, actuatorVal, initial=False, position=.2, max_effort=100.0):
        """
        This is a template for future creation of actions

        position (float): The position the gripper should go to (default=.2)
        max_effort (float): The force the gripper should apply during this process (default=100.0)
        """
        if initial:

            # this is the topic that the command should be published on
            # to perform the action
            # $(rostopic list) issued during a running scenario will
            # list all of the topics that can be published to
            topic='r_gripper_controller/command'

            # this is the command type that is published on the topic
            # $(rostopic info $topic) when used with a topic of your
            # choice will list the message type to use
            messageType='Pr2GripperCommand'

            # this creates the publisher on the given topic with the
            # given message type
            self.templateAction=rospy.Publisher(topic, eval(messageType))

            # this creates a new message with the given type
            # general form:
            # self.templateMessage=eval(messageType+'()')
            # nongeneral form:
            self.templateMessage=Pr2GripperCommand()

            # here you can set the attributes of the command to send
            # we grap the attributes from the arguments
            # nongeneral form (there is no general form):
            self.templateMessage.position=position
            self.templateMessage.max_effort=max_effort
        else:
            if int(actuatorVal)==1:
                # publish the message on the publisher created earlier
                # a note: the message will only work if it has time to
                # to complete the task before locomotion commands begin
                # to send again.  Try the goal oriented action function
                # if you want better completions
                self.templateAction.publish(self.templateMessage)


    def pickUpObject(self, actuatorVal, initial=False):
        """
        Pick Up Object with the Gripper and sensor on pr2
        """
        if initial:
            #rospy.init_node('pick_and_place_demo', anonymous=False)
            use_slip_controller = rospy.get_param('/reactive_grasp_node_right/use_slip_controller', 0)
            use_slip_detection = rospy.get_param('/reactive_grasp_node_right/use_slip_detection', 0)
            self.pick_and_place_demo = PickAndPlaceDemo(use_slip_controller = use_slip_controller, \
                                                       use_slip_detection = use_slip_detection)
        else:
            if int(actuatorVal)==1:
                #subprocess.Popen(['rosrun', 'pr2_pick_and_place_demos', 'IROS_2013_pick.py'],stdout=subprocess.PIPE)
                torso_action_client = actionlib.SimpleActionClient('torso_controller/position_joint_action', SingleJointPositionAction);
                rospy.loginfo("run_pick_and_place_demo: waiting for torso action server")
                torso_action_client.wait_for_server()
                rospy.loginfo("run_pick_and_place_demo: torso action server found")
                goal = SingleJointPositionGoal()
                goal.position = 0.295
                rospy.loginfo("sending command to lift torso")
                torso_action_client.send_goal(goal)
                torso_action_client.wait_for_result(rospy.Duration(30))

                #start the pick and place demo
                rospy.loginfo("starting pick and place demo")
                self.pick_and_place_demo.run_pick_up()
                print "PICK UP COMPLETED"
                #self.pick_and_place_demo.start_autonomous_thread(True)
                #pose = self.pose_handler.getPose()
                #self.drive_handler.setVelocity(self.Velocity[0,0], self.Velocity[1,0],pose[2])
                self.loco.sendCommand([-1,0,0])
                time.sleep(3)
                self.loco.sendCommand([0,0,0])

    def putDownObject(self, actuatorVal, initial=False,position=.6, max_effort=50.0):
        """
        Put down Object in the Gripper
        position (float): The position the gripper should go to (default=.6)
        max_effort (float): The force the gripper should apply during this process (default=50.0)
        """
        if initial:

            topicLeft='l_gripper_controller/command'
            topicRight='r_gripper_controller/command'
            messageType='Pr2GripperCommand'
            self.gripActionLeft=rospy.Publisher(topicLeft, eval(messageType))
            self.gripActionRight=rospy.Publisher(topicRight, eval(messageType))
            self.gripMessage=Pr2GripperCommand()
            self.gripMessage.position=position
            self.gripMessage.max_effort=max_effort



        else:
            if int(actuatorVal)==1:
                #action_name = 'tuck_arms'
                #tuck_arms_action_server = tuck_arms_main.TuckArmsActionServer(action_name)
                #goal = TuckArmsGoal()
                #goal.tuck_left = False
                #goal.tuck_right = False
                #tuck_arm_client = actionlib.SimpleActionClient(action_name, TuckArmsAction)
                #rospy.logdebug('Waiting for action server to start')
                #tuck_arm_client.wait_for_server(rospy.Duration(10.0))
                #rospy.logdebug('Sending goal to action server')
                #tuck_arm_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0))
                #self.loco.sendCommand([0,0,0])
                #time.sleep(15)
                #subprocess.Popen(['rosrun', 'pr2_tuckarm', 'tuck_arms.py','-r','u','-l','u'],stdout=subprocess.PIPE)
                #time.sleep(10)
                self.gripActionLeft.publish(self.gripMessage)
                self.gripActionRight.publish(self.gripMessage)
                time.sleep(20)


    def movePr2Gripper(self, actuatorVal, initial=False, whichHand='right', position=.2, max_effort=100.0):
        """
        This is a sample function for actuation of the Pr2's gripper

        whichHand (str): Specification of either the right or left gripper (default="right")
        position (float): The position the gripper should go to (default=.2)
        max_effort (float): The force the gripper should apply during this process (default=100.0)
        """
        if initial:

            if whichHand=='left':
                topic='l_gripper_controller/command'
            else:
                topic='r_gripper_controller/command'
            messageType='Pr2GripperCommand'
            self.gripAction=rospy.Publisher(topic, eval(messageType))
            self.gripMessage=Pr2GripperCommand()
            self.gripMessage.position=position
            self.gripMessage.max_effort=max_effort
        else:
            if int(actuatorVal)==1:
                self.gripAction.publish(self.gripMessage)


    def _templateGoalAction(self, actuatorVal, position=0.2, initial=False):
        """
        This is a template for a goal oriented action function

        position (float): The goal position of the torso (default=0.2)
        """
        if initial:
            # this is the topic you want to publish the message on
            # $(rostopic list) during a running session will list
            # the available topics
            topic='torso_controller/position_joint_action'

            # for goals, the messageType has two parts, the Action
            # and the Goal, in most cases it will be
            # messageType+'Action' for the action part and
            # messageType+'Goal' for the goal part
            messageType='SingleJointPosition'

            # here we create the action/goal client with the Action
            # messageType
            self.templateAction = actionlib.SimpleActionClient(topic, eval(messageType+'Action'))

            # this is the creation of the message, notice the message
            # has a Goal format
            self.templateMessage=eval(messageType+'Goal()')

            # set the goal attributes of the message, here we just
            # have position (torso)
            self.templateMessage.position=position
        else:
            if int(actuatorVal)==1:
                # bring up the client to allow for message sending
                self.templateAction.wait_for_server()

                # send the goal message
                self.templateAction.send_goal(self.templateMessage)

                # wait for a result
                self.templateAction.wait_for_result()

                # Don't burden the user with success messages
                # however, if the action fails, the user will want
                # to know
                if not templateAction.get_state() == GoalStatus.SUCCEEDED:
                    print "Action failed"

    def movePr2Torso(self, actuatorVal, position=0.2, initial=False):
        """
        This is a sample goal oriented actuation for moving the Pr2's torso

        position (float): The goal position of the torso (default=0.2)
        """
        if initial:
            topic='torso_controller/position_joint_action'
            messageType='SingleJointPosition'
            self.torsoAction = actionlib.SimpleActionClient(topic, eval(messageType+'Action'))
            self.torsoMessage=eval(messageType+'Goal()')
            self.torsoMessage.position=position
        else:
            if int(actuatorVal)==1:
                self.torsoAction.wait_for_server()
                self.torsoAction.send_goal(self.torsoMessage)
                self.torsoAction.wait_for_result()
                if not self.torsoAction.get_state() == GoalStatus.SUCCEEDED:
                    print "Action failed"

    def movePr2Head(self, actuatorVal, target_frame_id='r_gripper_tool_frame', target_x=0.0, target_y=0.0, target_z=0.0, min_duration=1.0, initial=False):
        """
        This is a sample goal oriented actuation for moving the Pr2's head

        target_frame_id (str): The frame of refernce you want to look at (default='r_gripper_tool_frame')
        target_x (float): The x position in the frame of reference (default=0.0)
        target_y (float): The y position in the frame of reference (default=0.0)
        target_z (float): The z position in the frame of reference (default=0.0)
        min_duration (float): The minimum movement time (default=1.0)
        """
        if initial:
            topic='head_traj_controller/point_head_action'
            messageType='PointHead'
            self.headAction = actionlib.SimpleActionClient(topic, eval(messageType+'Action'))
            self.headMessage=eval(messageType+'Goal()')
            self.headMessage.target.header.frame_id=target_frame_id
            # for a list of frame_ids: http://www.ros.org/wiki/pr2_description
            self.headMessage.target.point.x=target_x
            self.headMessage.target.point.y=target_y
            self.headMessage.target.point.z=target_z
            self.headMessage.min_duration=rospy.Duration(min_duration)
        else:
            if int(actuatorVal)==1:
                self.headAction.wait_for_server()
                self.headAction.send_goal(self.headMessage)
                self.headAction.wait_for_result()
                if not self.headAction.get_state() == GoalStatus.SUCCEEDED:
                    print "Action failed"

    def pr2TuckArms(self, actuatorVal, tuck=True, initial=False):
        """
        This is an example of calling existing scripts with ROS

        tuck (bool): True applies a tucking of the arms, False untucks them (default=True)
        """
        if initial:
            pass
        else:
            if int(actuatorVal)==1:
                if tuck:
                    subprocess.Popen(['rosrun', 'pr2_tuckarm', 'tuck_arms.py','-r','t','-l','t'],stdout=subprocess.PIPE)
                else:
                    subprocess.Popen(['rosrun', 'pr2_tuckarm', 'tuck_arms.py','-r','u','-l','u'],stdout=subprocess.PIPE)
