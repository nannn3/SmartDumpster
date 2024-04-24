import math
import time
import numpy as np
from scipy.spatial.transform import Rotation
from franky import JointWaypointMotion, JointWaypoint, JointPositionStopMotion, CartesianMotion, CartesianWaypointMotion, CartesianWaypoint, Affine, RobotPose, ReferenceType, CartesianPoseStopMotion
from franky import Robot
from franky import Gripper

#camera alignment

#reconfigure these values
TL_Cam=(159,126)
TR_Cam=(409,27)
BL_Cam=(239,315)
BR_Cam=(519,183)


TL_Bot=(0.4,-0.2)
TR_Bot=(0.4,0.2)
BL_Bot=(0.7,-0.2)
BR_Bot=(0.7,0.2)


Bot_X_in=np.array([[TL_Cam[0]*TL_Cam[1], TL_Cam[0], TL_Cam[1], 1],
		   [TR_Cam[0]*TR_Cam[1], TR_Cam[0], TR_Cam[1], 1],
		   [BL_Cam[0]*BL_Cam[1], BL_Cam[0], BL_Cam[1], 1],
		   [BR_Cam[0]*BR_Cam[1], BR_Cam[0], BR_Cam[1], 1]
		   ])
Bot_X_out=np.array([TL_Bot[0], TR_Bot[0], BL_Bot[0], BR_Bot[0]])
a_X_Bot, b_X_Bot, c_X_Bot, d_X_Bot=np.linalg.solve(Bot_X_in, Bot_X_out)


Bot_Y_in=np.array([[TL_Cam[0]*TL_Cam[1], TL_Cam[0], TL_Cam[1], 1],
		   [TR_Cam[0]*TR_Cam[1], TR_Cam[0], TR_Cam[1], 1],
		   [BL_Cam[0]*BL_Cam[1], BL_Cam[0], BL_Cam[1], 1],
		   [BR_Cam[0]*BR_Cam[1], BR_Cam[0], BR_Cam[1], 1]
		   ])
Bot_Y_out=np.array([TL_Bot[1], TR_Bot[1], BL_Bot[1], BR_Bot[1]])
a_Y_Bot, b_Y_Bot, c_Y_Bot, d_Y_Bot=np.linalg.solve(Bot_Y_in, Bot_Y_out)


print(a_Y_Bot)
print(b_Y_Bot)
print(c_Y_Bot)
print(d_Y_Bot)


robot = Robot("172.16.0.2")
gripper= Gripper("172.16.0.2",0.3)


# Recover from errors
robot.recover_from_errors()

# Set velocity, acceleration and jerk to 5% of the maximum
robot.relative_dynamics_factor = 0.7

gripper.move_async(0.08)

# Get the current pose
current_pose = robot.current_pose


"""
# A point-to-point motion in the joint space
m1 = JointWaypointMotion([JointWaypoint([-1.8, 1.1, 1.7, -2.1, -1.1, 1.6, -0.4])])

# A motion in joint space with multiple waypoints
m2 = JointWaypointMotion([
    JointWaypoint([-1.8, 1.1, 1.7, -2.1, -1.1, 1.6, -0.4]),
    JointWaypoint([-1.7, 1.2, 1.8, -2.0, -1.0, 1.7, -0.3]),
    JointWaypoint([-1.9, 1.0, 1.6, -2.2, -1.2, 1.5, -0.5])
])

# Stop the robot
m3 = JointPositionStopMotion()

# A linear motion in cartesian space
quat = Rotation.from_euler("xyz", [0, 0, math.pi / 2]).as_quat()
m4 = CartesianMotion(Affine([0.2, -0.4, 0.3], quat))
m5 = CartesianMotion(RobotPose(Affine([0.2, -0.4, 0.3], quat), elbow_position=1.7)) # With target elbow angle

# A linear motion in cartesian space relative to the initial position
# (Note that this motion is relative both in position and orientation. Hence, when the robot's end-effector is oriented
# differently, it will move in a different direction)
quat = Rotation.from_euler("xyz", [-math.pi, 0, 0]).as_quat()
m6 = CartesianMotion(Affine([0.5, 0.0, 0.4],quat), ReferenceType.Absolute)

# Generalization of CartesianMotion that allows for multiple waypoints
m7 = CartesianWaypointMotion([
  CartesianWaypoint(RobotPose(Affine([0.2, -0.4, 0.3], quat), elbow_position=1.7)),
  # The following waypoint is relative to the prior one and 50% slower
  CartesianWaypoint(Affine([0.2, 0.0, 0.0]), ReferenceType.Relative, velocity_rel=0.5)
])

# Stop the robot. The difference of JointPositionStopMotion to CartesianPoseStopMotion is that JointPositionStopMotion
# stops the robot in joint position control mode while CartesianPoseStopMotion stops it in cartesian pose control mode.
# The difference becomes relevant when asynchronous move commands are being sent (see below).
m8 = CartesianPoseStopMotion()
"""
quat = Rotation.from_euler("xyz", [-math.pi, 0, 0]).as_quat()
m6 = CartesianMotion(Affine([0.5, 0.0, 0.25],quat), ReferenceType.Absolute)

robot.move(m6)

STARTfile=open("items/START.txt", "r")
FINISHEDfile = open("items/Finished_Items.txt", "a")

nextline=0

currentpending=""

while(True):
	STARTfile=open("items/START.txt", "r")
	if(len(STARTfile.read())>2):
		STARTfile.close()
		STARTfile = open("items/START.txt", "w")
		STARTfile.write("")
		STARTfile.close()
		pending = open("items/Pending_Items_Camera.txt", "r")
		currentpending=pending.read()
		pending.close()
		#pending = open("items/Pending_Items_Camera.txt", "w")
		#pending.write("")
		#pending.close()
		nextline=0
		
		
		print(currentpending)
		"""
	if(len(currentpending)>2 and len(currentpending)>nextline):
		print(nextline)
		comma1=currentpending.find(',',nextline)
		Xnext=float(currentpending[nextline:comma1])
		print(Xnext)
		comma2=currentpending.find(',',comma1+1)
		Ynext=float(currentpending[comma1+1:comma2])
		print(Ynext)
		pipe=currentpending.find('|',nextline)
		Znext=float(currentpending[comma2+1:pipe])
		print(Znext)
		nextline=pipe+1
		FINISHEDfile.write(str(Xnext) + "," + str(Ynext) + "," + str(Znext) + "|\n")
		
		m6 = CartesianWaypointMotion([CartesianWaypoint(Affine([0.5, 0.0, 0.2],quat), ReferenceType.Absolute),
		CartesianWaypoint(Affine([Xnext, Ynext, 0.1],quat), ReferenceType.Absolute),
		CartesianWaypoint(Affine([Xnext, Ynext, Znext],quat), ReferenceType.Absolute)
		])

		robot.move(m6)
		
		gripper.clamp()
		
		
		m6 = CartesianWaypointMotion([CartesianWaypoint(Affine([Xnext, Ynext, 0.1],quat), ReferenceType.Absolute),
		CartesianWaypoint(Affine([0.5, 0.0, 0.2],quat), ReferenceType.Absolute),
		CartesianWaypoint(Affine([0.0, 0.5, 0.25],quat), ReferenceType.Absolute)
		])
		robot.move(m6)
		
		gripper.move_async(0.08)
		"""
	if(len(currentpending)>2 and len(currentpending)>nextline):
		print(len(currentpending))
		print(nextline)
		comma1=currentpending.find(',',nextline)
		Xcam=float(currentpending[nextline:comma1])
		print(Xcam)
		pipe=currentpending.find('|',comma1+1)
		Ycam=float(currentpending[comma1+1:pipe])
		print(Ycam)
		nextline=pipe+2
		#FINISHEDfile.write(str(Xnext) + "," + str(Ynext) + "," + str(Znext) + "|\n")
		Xnext=(a_X_Bot*Xcam*Ycam+b_X_Bot*Xcam+c_X_Bot*Ycam+d_X_Bot)
		Ynext=(a_Y_Bot*Xcam*Ycam+b_Y_Bot*Xcam+c_Y_Bot*Ycam+d_Y_Bot)
		
		print(Xnext)
		print(Ynext)
		m6 = CartesianWaypointMotion([CartesianWaypoint(Affine([0.5, 0.0, 0.2],quat), ReferenceType.Absolute),
		CartesianWaypoint(Affine([Xnext, Ynext, 0.1],quat), ReferenceType.Absolute),
		CartesianWaypoint(Affine([Xnext, Ynext, 0.04],quat), ReferenceType.Absolute)
		])

		robot.move(m6)
		
		gripper.clamp()
		
		
		m6 = CartesianWaypointMotion([CartesianWaypoint(Affine([Xnext, Ynext, 0.1],quat), ReferenceType.Absolute),
		CartesianWaypoint(Affine([0.5, 0.0, 0.2],quat), ReferenceType.Absolute),
		CartesianWaypoint(Affine([0.0, 0.5, 0.25],quat), ReferenceType.Absolute)
		])
		robot.move(m6)
		
		gripper.move_async(0.08)
		
		
		
		time.sleep(0.7)
	
	
