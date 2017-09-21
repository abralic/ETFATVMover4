#! /usr/bin/env python

from __future__ import print_function

import rospy
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from std_msgs.msg import String
from mavros_msgs.msg import RCIn
from sensor_msgs.msg import JointState

class Mover4:
	
	def __init__(self):
		self.upperThreshold = 2000
		self.lowerThreshold = 990
		self.gripperRadioChannelNumber = 8
		self.baseRadioChannelNumber = 9
		self.shoulderRadioChannelNumber = 10
		self.elbowRadioChannelNumber = 11
		self.wristRadioChannelNumber = 12
		self.maxEffort = 10
		self.gripper_opened = False
		self.is_running = False
		self.gripperGoal = None
		self.jointVelocity = None
		self.baseVelocityValue = None
		self.shoulderVelocityValue = None
		self.elbowVelocityValue = None
		self.wristVelocityValue = None
		self.jointStates = None
		self.toDeg = 180/3.14
		
		self.mover4_error_codes = rospy.Subscriber('/CPRMoverErrorCodes', String, self.error_codes_callback, queue_size = 10)
		self.init_mover4()
		rospy.loginfo("Mover4 initialized.")
		self.gripper_client = actionlib.SimpleActionClient('cpr_mover/gripper_command', GripperCommandAction)
		self.gripper_client.wait_for_server()
		rospy.loginfo("GripperServer available.")
		self.joint_velocity_publisher = rospy.Publisher('/CPRMoverJointVel', JointState, queue_size = 10)
		self.rc_in_subscriber = rospy.Subscriber('mavros/rc/in', RCIn, self.rc_in_callback, queue_size = 10)
		self.joint_states_subscriber = rospy.Subscriber('/joint_states', JointState, self.joint_states_callback, queue_size = 10)
	
	def init_mover4(self):
		mover4_publisher = rospy.Publisher('/CPRMoverCommands', String, queue_size = 10)
		rate = rospy.Rate(10)
		if not self.is_running:
			rospy.loginfo("Connecting and Enabling Mover4")
			msg1 = "Connect"
			mover4_publisher.publish(msg1)
			rospy.sleep(1.)
			mover4_publisher.publish(msg1)
			rospy.sleep(1.)
			msg2 = "Reset"
			mover4_publisher.publish(msg2)
			rospy.sleep(1.)
			msg3 = "Enable"
			mover4_publisher.publish(msg3)
			rospy.sleep(1.)
		else:
			rospy.loginfo("Already connected and enabled Mover4")

	def open_gripper(self):
		return 100

	def close_gripper(self):
		return -100
	
	def set_joint_velocity(self, rcValue):
		if rcValue > 1900:
			return 80		
		elif (rcValue <= 1900 and rcValue > 1650):
			return 40
		elif (rcValue <= 1650 and rcValue > 1334):
			return 0
		elif (rcValue <= 1334 and rcValue > 1000):
			return -40
		elif (rcValue <= 1000):
			return -80
	
	def compute_base_velocity(self, vel):
		if self.jointStates.position[0]*self.toDeg > 130 and vel > 0:
			rospy.loginfo("MAXIMUM REACHED!")
			return 0
		elif self.jointStates.position[0]*self.toDeg > 130 and vel < 0:
			return vel
		elif self.jointStates.position[0]*self.toDeg < -130 and vel < 0:
			rospy.loginfo("MINIMUM REACHED!")
			return 0
		elif self.jointStates.position[0]*self.toDeg < -130 and vel > 0:
			return vel
		else:	
			return vel
	
	def compute_shoulder_velocity(self, vel):
		if self.jointStates.position[1]*self.toDeg > 55 and vel > 0:
			rospy.loginfo("MAXIMUM REACHED!")
			return 0
		elif self.jointStates.position[1]*self.toDeg > 55 and vel < 0:
			return vel
		elif self.jointStates.position[1]*self.toDeg < -20 and vel < 0:
			rospy.loginfo("MINIMUM REACHED!")
			return 0
		elif self.jointStates.position[1]*self.toDeg < -20 and vel > 0:
			return vel
		else:	
			return vel

	def compute_elbow_velocity(self, vel):
		if self.jointStates.position[2]*self.toDeg > 120 and vel > 0:
			rospy.loginfo("MAXIMUM REACHED!")
			return 0
		elif self.jointStates.position[2]*self.toDeg > 120 and vel < 0:
			return vel
		elif self.jointStates.position[2]*self.toDeg < -30 and vel < 0:
			rospy.loginfo("MINIMUM REACHED!")
			return 0
		elif self.jointStates.position[2]*self.toDeg < -30 and vel > 0:
			return vel
		else:	
			return vel

	def compute_wrist_velocity(self, vel):
		if self.jointStates.position[3]*self.toDeg > 120 and vel > 0:
			rospy.loginfo("MAXIMUM REACHED!")
			return 0
		elif self.jointStates.position[3]*self.toDeg > 120 and vel < 0:
			return vel
		elif self.jointStates.position[3]*self.toDeg < -120 and vel < 0:
			rospy.loginfo("MINIMUM REACHED!")
			return 0
		elif self.jointStates.position[3]*self.toDeg < -120 and vel > 0:
			return vel
		else:	
			return vel

	def move_joints(self, rcIn):
		self.jointVelocity = JointState()
		
		self.baseVelocityValue = self.compute_base_velocity(self.set_joint_velocity(rcIn.channels[self.baseRadioChannelNumber]))
		self.shoulderVelocityValue = self.compute_shoulder_velocity(self.set_joint_velocity(rcIn.channels[self.shoulderRadioChannelNumber]))
		self.elbowVelocityValue = self.compute_elbow_velocity(self.set_joint_velocity(rcIn.channels[self.elbowRadioChannelNumber]))
		self.wristVelocityValue = self.compute_wrist_velocity(self.set_joint_velocity(rcIn.channels[self.wristRadioChannelNumber]))		

		self.jointVelocity.velocity = [self.baseVelocityValue, self.shoulderVelocityValue, self.elbowVelocityValue, self.wristVelocityValue]
		return self.jointVelocity

	def error_codes_callback(self, message):
		if "running" in message.data:
			if self.gripperGoal is not None:			
				self.is_running = True
		else:
			return
	
	def joint_states_callback(self, jointStates):
		self.jointStates = jointStates	

	def rc_in_callback(self, rcIn):
		
		self.gripperGoal = GripperCommandGoal()
		
		if rcIn.channels[self.gripperRadioChannelNumber] > self.upperThreshold:
			self.gripperGoal.command.position = self.open_gripper()
			self.gripper_opened = True
		
			rospy.loginfo("Gripper opened.")
		elif rcIn.channels[self.gripperRadioChannelNumber] < self.lowerThreshold:
			self.gripperGoal.command.position = self.close_gripper()
			self.gripper_opened = False
			
			rospy.loginfo("Gripper closed.")
		else:
			
			if self.gripper_opened:
				self.gripperGoal.command.position = self.open_gripper()
				self.gripper_opened = True
			rospy.loginfo("No gripper action.")

		self.gripper_client.send_goal(self.gripperGoal)
		self.gripper_client.wait_for_result()
		#rospy.loginfo("Gripper RC commmand value: %d", rcIn.channels[self.gripperRadioChannelNumber])
		
		
		self.joint_velocity_publisher.publish(self.move_joints(rcIn))

		rospy.loginfo("Base position and velocity:     %f   %f", self.jointStates.position[0] * self.toDeg, self.baseVelocityValue)
		rospy.loginfo("Shoulder position and velocity: %f	%f", self.jointStates.position[1] * self.toDeg, self.shoulderVelocityValue)
		rospy.loginfo("Elbow position and velocity:    %f	%f", self.jointStates.position[2] * self.toDeg, self.elbowVelocityValue)
		rospy.loginfo("Wrist position and velocity:    %f	%f", self.jointStates.position[3] * self.toDeg, self.wristVelocityValue)
		rospy.loginfo("\n")

if __name__ == '__main__':
		try:
			rospy.init_node('mover4_teleop_py')
			gn = Mover4()
			rospy.spin()
		except rospy.ROSInterruptException:
			print("Interrupted!")
