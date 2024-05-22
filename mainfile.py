import math
import time
import ast
import numpy as np
from scipy.spatial.transform import Rotation
from franky import JointWaypointMotion, JointWaypoint, JointPositionStopMotion, CartesianMotion, CartesianWaypointMotion, CartesianWaypoint, Affine, RobotPose, ReferenceType, CartesianPoseStopMotion
from franky import Robot
from franky import Gripper

#camera alignment

#reconfigure these values
calibrationfile = open("items/Calibration.txt", "r")
currentcalibration=calibrationfile.read()
calibrationfile.close()
nline=0
numcal=0
bigdict={}
while(numcal<8):
	numcal+=1
	endofline=currentcalibration.find('\n',nline+1)

	caldict=ast.literal_eval(currentcalibration[nline:endofline])
	bigdict[caldict["Name"]]=caldict
	nline=endofline
	
#print(bigdict)
curcal=bigdict["Short_White"]
Small_White_Cam=(curcal["X_0"],curcal["Y_0"],curcal["Dep"])
curcal=bigdict["Short_Blue"]
Small_Blue_Cam=(curcal["X_0"],curcal["Y_0"],curcal["Dep"])
curcal=bigdict["Short_Green"]
Small_Green_Cam=(curcal["X_0"],curcal["Y_0"],curcal["Dep"])
curcal=bigdict["Short_Orange"]
Small_Orange_Cam=(curcal["X_0"],curcal["Y_0"],curcal["Dep"])

curcal=bigdict["Tall_Green"]
Tall_Green_Cam=(curcal["X_0"],curcal["Y_0"],curcal["Dep"])
curcal=bigdict["Tall_Orange"]
Tall_Orange_Cam=(curcal["X_0"],curcal["Y_0"],curcal["Dep"])
curcal=bigdict["Tall_White"]
Tall_White_Cam=(curcal["X_0"],curcal["Y_0"],curcal["Dep"])
curcal=bigdict["Tall_Blue"]
Tall_Blue_Cam=(curcal["X_0"],curcal["Y_0"],curcal["Dep"])

print(Tall_Green_Cam)
print(Tall_Orange_Cam)
print(Tall_White_Cam)
print(Tall_Blue_Cam)

Small_White_Bot=(0.3,0.15,0.21)
Small_Blue_Bot=(0.3,-0.05,0.21)
Small_Green_Bot=(0.79,-0.15,0.21)
Small_Orange_Bot=(0.79,0.05,0.21)

Tall_Green_Bot=(0.3,-0.15,0.25)
Tall_Orange_Bot=(0.3,0.05,0.25)
Tall_White_Bot=(0.79,0.15,0.25)
Tall_Blue_Bot=(0.79,-0.05,0.25)


Bot_X_in=np.array([[Small_White_Cam[0]*Small_White_Cam[1]*Small_White_Cam[2], Small_White_Cam[0]*Small_White_Cam[1],Small_White_Cam[1]*Small_White_Cam[2], Small_White_Cam[0]*Small_White_Cam[2], Small_White_Cam[0], Small_White_Cam[1], Small_White_Cam[2], 1],
		   [Small_Blue_Cam[0]*Small_Blue_Cam[1]*Small_Blue_Cam[2], Small_Blue_Cam[0]*Small_Blue_Cam[1],Small_Blue_Cam[1]*Small_Blue_Cam[2], Small_Blue_Cam[0]*Small_Blue_Cam[2], Small_Blue_Cam[0], Small_Blue_Cam[1], Small_Blue_Cam[2], 1],
		   [Small_Green_Cam[0]*Small_Green_Cam[1]*Small_Green_Cam[2], Small_Green_Cam[0]*Small_Green_Cam[1],Small_Green_Cam[1]*Small_Green_Cam[2], Small_Green_Cam[0]*Small_Green_Cam[2], Small_Green_Cam[0], Small_Green_Cam[1], Small_Green_Cam[2], 1],
		   [Small_Orange_Cam[0]*Small_Orange_Cam[1]*Small_Orange_Cam[2], Small_Orange_Cam[0]*Small_Orange_Cam[1],Small_Orange_Cam[1]*Small_Orange_Cam[2], Small_Orange_Cam[0]*Small_Orange_Cam[2], Small_Orange_Cam[0], Small_Orange_Cam[1], Small_Orange_Cam[2], 1],
		   [Tall_Green_Cam[0]*Tall_Green_Cam[1]*Tall_Green_Cam[2], Tall_Green_Cam[0]*Tall_Green_Cam[1],Tall_Green_Cam[1]*Tall_Green_Cam[2], Tall_Green_Cam[0]*Tall_Green_Cam[2], Tall_Green_Cam[0], Tall_Green_Cam[1], Tall_Green_Cam[2], 1],
		   [Tall_Orange_Cam[0]*Tall_Orange_Cam[1]*Tall_Orange_Cam[2], Tall_Orange_Cam[0]*Tall_Orange_Cam[1],Tall_Orange_Cam[1]*Tall_Orange_Cam[2], Tall_Orange_Cam[0]*Tall_Orange_Cam[2], Tall_Orange_Cam[0], Tall_Orange_Cam[1], Tall_Orange_Cam[2], 1],
		   [Tall_White_Cam[0]*Tall_White_Cam[1]*Tall_White_Cam[2], Tall_White_Cam[0]*Tall_White_Cam[1],Tall_White_Cam[1]*Tall_White_Cam[2], Tall_White_Cam[0]*Tall_White_Cam[2], Tall_White_Cam[0], Tall_White_Cam[1], Tall_White_Cam[2], 1],
		   [Tall_Blue_Cam[0]*Tall_Blue_Cam[1]*Tall_Blue_Cam[2], Tall_Blue_Cam[0]*Tall_Blue_Cam[1],Tall_Blue_Cam[1]*Tall_Blue_Cam[2], Tall_Blue_Cam[0]*Tall_Blue_Cam[2], Tall_Blue_Cam[0], Tall_Blue_Cam[1], Tall_Blue_Cam[2], 1]
		   
		   ])
Bot_X_out=np.array([Small_White_Bot[0], Small_Blue_Bot[0], Small_Green_Bot[0], Small_Orange_Bot[0],Tall_Green_Bot[0], Tall_Orange_Bot[0], Tall_White_Bot[0], Tall_Blue_Bot[0]])
a_X_Bot, b_X_Bot, c_X_Bot, d_X_Bot, e_X_Bot, f_X_Bot, g_X_Bot, h_X_Bot=np.linalg.solve(Bot_X_in, Bot_X_out)


Bot_Y_in=np.array([[Small_White_Cam[0]*Small_White_Cam[1]*Small_White_Cam[2], Small_White_Cam[0]*Small_White_Cam[1],Small_White_Cam[1]*Small_White_Cam[2], Small_White_Cam[0]*Small_White_Cam[2], Small_White_Cam[0], Small_White_Cam[1], Small_White_Cam[2], 1],
		   [Small_Blue_Cam[0]*Small_Blue_Cam[1]*Small_Blue_Cam[2], Small_Blue_Cam[0]*Small_Blue_Cam[1],Small_Blue_Cam[1]*Small_Blue_Cam[2], Small_Blue_Cam[0]*Small_Blue_Cam[2], Small_Blue_Cam[0], Small_Blue_Cam[1], Small_Blue_Cam[2], 1],
		   [Small_Green_Cam[0]*Small_Green_Cam[1]*Small_Green_Cam[2], Small_Green_Cam[0]*Small_Green_Cam[1],Small_Green_Cam[1]*Small_Green_Cam[2], Small_Green_Cam[0]*Small_Green_Cam[2], Small_Green_Cam[0], Small_Green_Cam[1], Small_Green_Cam[2], 1],
		   [Small_Orange_Cam[0]*Small_Orange_Cam[1]*Small_Orange_Cam[2], Small_Orange_Cam[0]*Small_Orange_Cam[1],Small_Orange_Cam[1]*Small_Orange_Cam[2], Small_Orange_Cam[0]*Small_Orange_Cam[2], Small_Orange_Cam[0], Small_Orange_Cam[1], Small_Orange_Cam[2], 1],
		   [Tall_Green_Cam[0]*Tall_Green_Cam[1]*Tall_Green_Cam[2], Tall_Green_Cam[0]*Tall_Green_Cam[1],Tall_Green_Cam[1]*Tall_Green_Cam[2], Tall_Green_Cam[0]*Tall_Green_Cam[2], Tall_Green_Cam[0], Tall_Green_Cam[1], Tall_Green_Cam[2], 1],
		   [Tall_Orange_Cam[0]*Tall_Orange_Cam[1]*Tall_Orange_Cam[2], Tall_Orange_Cam[0]*Tall_Orange_Cam[1],Tall_Orange_Cam[1]*Tall_Orange_Cam[2], Tall_Orange_Cam[0]*Tall_Orange_Cam[2], Tall_Orange_Cam[0], Tall_Orange_Cam[1], Tall_Orange_Cam[2], 1],
		   [Tall_White_Cam[0]*Tall_White_Cam[1]*Tall_White_Cam[2], Tall_White_Cam[0]*Tall_White_Cam[1],Tall_White_Cam[1]*Tall_White_Cam[2], Tall_White_Cam[0]*Tall_White_Cam[2], Tall_White_Cam[0], Tall_White_Cam[1], Tall_White_Cam[2], 1],
		   [Tall_Blue_Cam[0]*Tall_Blue_Cam[1]*Tall_Blue_Cam[2], Tall_Blue_Cam[0]*Tall_Blue_Cam[1],Tall_Blue_Cam[1]*Tall_Blue_Cam[2], Tall_Blue_Cam[0]*Tall_Blue_Cam[2], Tall_Blue_Cam[0], Tall_Blue_Cam[1], Tall_Blue_Cam[2], 1]
		   
		   ])
Bot_Y_out=np.array([Small_White_Bot[1], Small_Blue_Bot[1], Small_Green_Bot[1], Small_Orange_Bot[1], Tall_Green_Bot[1], Tall_Orange_Bot[1], Tall_White_Bot[1], Tall_Blue_Bot[1]])


a_Y_Bot, b_Y_Bot, c_Y_Bot, d_Y_Bot,e_Y_Bot, f_Y_Bot, g_Y_Bot, h_Y_Bot=np.linalg.solve(Bot_Y_in, Bot_Y_out)

Bot_Z_in=np.array([[Small_White_Cam[0]*Small_White_Cam[1]*Small_White_Cam[2], Small_White_Cam[0]*Small_White_Cam[1],Small_White_Cam[1]*Small_White_Cam[2], Small_White_Cam[0]*Small_White_Cam[2], Small_White_Cam[0], Small_White_Cam[1], Small_White_Cam[2], 1],
		   [Small_Blue_Cam[0]*Small_Blue_Cam[1]*Small_Blue_Cam[2], Small_Blue_Cam[0]*Small_Blue_Cam[1],Small_Blue_Cam[1]*Small_Blue_Cam[2], Small_Blue_Cam[0]*Small_Blue_Cam[2], Small_Blue_Cam[0], Small_Blue_Cam[1], Small_Blue_Cam[2], 1],
		   [Small_Green_Cam[0]*Small_Green_Cam[1]*Small_Green_Cam[2], Small_Green_Cam[0]*Small_Green_Cam[1],Small_Green_Cam[1]*Small_Green_Cam[2], Small_Green_Cam[0]*Small_Green_Cam[2], Small_Green_Cam[0], Small_Green_Cam[1], Small_Green_Cam[2], 1],
		   [Small_Orange_Cam[0]*Small_Orange_Cam[1]*Small_Orange_Cam[2], Small_Orange_Cam[0]*Small_Orange_Cam[1],Small_Orange_Cam[1]*Small_Orange_Cam[2], Small_Orange_Cam[0]*Small_Orange_Cam[2], Small_Orange_Cam[0], Small_Orange_Cam[1], Small_Orange_Cam[2], 1],
		   [Tall_Green_Cam[0]*Tall_Green_Cam[1]*Tall_Green_Cam[2], Tall_Green_Cam[0]*Tall_Green_Cam[1],Tall_Green_Cam[1]*Tall_Green_Cam[2], Tall_Green_Cam[0]*Tall_Green_Cam[2], Tall_Green_Cam[0], Tall_Green_Cam[1], Tall_Green_Cam[2], 1],
		   [Tall_Orange_Cam[0]*Tall_Orange_Cam[1]*Tall_Orange_Cam[2], Tall_Orange_Cam[0]*Tall_Orange_Cam[1],Tall_Orange_Cam[1]*Tall_Orange_Cam[2], Tall_Orange_Cam[0]*Tall_Orange_Cam[2], Tall_Orange_Cam[0], Tall_Orange_Cam[1], Tall_Orange_Cam[2], 1],
		   [Tall_White_Cam[0]*Tall_White_Cam[1]*Tall_White_Cam[2], Tall_White_Cam[0]*Tall_White_Cam[1],Tall_White_Cam[1]*Tall_White_Cam[2], Tall_White_Cam[0]*Tall_White_Cam[2], Tall_White_Cam[0], Tall_White_Cam[1], Tall_White_Cam[2], 1],
		   [Tall_Blue_Cam[0]*Tall_Blue_Cam[1]*Tall_Blue_Cam[2], Tall_Blue_Cam[0]*Tall_Blue_Cam[1],Tall_Blue_Cam[1]*Tall_Blue_Cam[2], Tall_Blue_Cam[0]*Tall_Blue_Cam[2], Tall_Blue_Cam[0], Tall_Blue_Cam[1], Tall_Blue_Cam[2], 1]
		   
		   ])
Bot_Z_out=np.array([Small_White_Bot[2], Small_Blue_Bot[2], Small_Green_Bot[2], Small_Orange_Bot[2], Tall_Green_Bot[2], Tall_Orange_Bot[2], Tall_White_Bot[2], Tall_Blue_Bot[2]])

a_Z_Bot, b_Z_Bot, c_Z_Bot, d_Z_Bot,e_Z_Bot, f_Z_Bot, g_Z_Bot, h_Z_Bot=np.linalg.solve(Bot_Z_in, Bot_Z_out)

"""
curtest=Tall_Green_Cam
Xcam=432
Ycam=148
Zdep=0.8628109283745289
Xtest=(a_X_Bot*Xcam*Ycam*Zdep+b_X_Bot*Xcam*Ycam+c_X_Bot*Ycam*Zdep+d_X_Bot*Xcam*Zdep+e_X_Bot*Xcam+f_X_Bot*Ycam+g_X_Bot*Zdep+h_X_Bot)
Ytest=(a_Y_Bot*Xcam*Ycam*Zdep+b_Y_Bot*Xcam*Ycam+c_Y_Bot*Ycam*Zdep+d_Y_Bot*Xcam*Zdep+e_Y_Bot*Xcam+f_Y_Bot*Ycam+g_Y_Bot*Zdep+h_Y_Bot)	
Ztest=(a_Z_Bot*Xcam*Ycam*Zdep+b_Z_Bot*Xcam*Ycam+c_Z_Bot*Ycam*Zdep+d_Z_Bot*Xcam*Zdep+e_Z_Bot*Xcam+f_Z_Bot*Ycam+g_Z_Bot*Zdep+h_Z_Bot)
"""



robot = Robot("172.16.0.2")
gripper= Gripper("172.16.0.2",0.3,50) #sets force and speed thresholds


# Recover from errors
robot.recover_from_errors()

# Set velocity, acceleration and jerk to 5% of the maximum
robot.relative_dynamics_factor = 0.5 #sets speed
#robot.velocity_rel = 0.05
#robot.acceleration_rel = 0.05
#robot.jerk_rel = 0.05

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
angle=0
quat = Rotation.from_euler("xyz", [-math.pi, 0, angle]).as_quat()
m6 = CartesianMotion(Affine([0.5, 0.0, 0.6],quat), ReferenceType.Absolute) #moves to start position out of camera frame

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
		pending = open("items/Items_Rot_Dep.txt", "r")
		currentpending=pending.read()
		pending.close()
		pending = open("items/Items_Rot_Dep.txt", "w")
		pending.write("")
		pending.close()
		nextline=0
		
		

	if(len(currentpending)>2 and len(currentpending)>nextline):
		quat = Rotation.from_euler("xyz", [-math.pi, 0, angle]).as_quat()
		
		endoftheline=currentpending.find('\n',nextline)
		posdict=ast.literal_eval(currentpending[nextline:endoftheline])
	
		Xcam=posdict["X_0"]
		Ycam=posdict["Y_0"]
		Zdep=posdict["Dep"]
		nextline=endoftheline+1
		#FINISHEDfile.write(str(Xnext) + "," + str(Ynext) + "," + str(Znext) + "|\n")
		Xnext=(a_X_Bot*Xcam*Ycam*Zdep+b_X_Bot*Xcam*Ycam+c_X_Bot*Ycam*Zdep+d_X_Bot*Xcam*Zdep+e_X_Bot*Xcam+f_X_Bot*Ycam+g_X_Bot*Zdep+h_X_Bot)
		Ynext=(a_Y_Bot*Xcam*Ycam*Zdep+b_Y_Bot*Xcam*Ycam+c_Y_Bot*Ycam*Zdep+d_Y_Bot*Xcam*Zdep+e_Y_Bot*Xcam+f_Y_Bot*Ycam+g_Y_Bot*Zdep+h_Y_Bot)
		Znext=(a_Z_Bot*Xcam*Ycam*Zdep+b_Z_Bot*Xcam*Ycam+c_Z_Bot*Ycam*Zdep+d_Z_Bot*Xcam*Zdep+e_Z_Bot*Xcam+f_Z_Bot*Ycam+g_Z_Bot*Zdep+h_Z_Bot)
		
		print(Xnext)
		print(Ynext)
		print(Znext)
		
		#print(Xnext)
		#print(Ynext)
		#print(a_Z_Bot,b_Z_Bot,c_Z_Bot,d_Z_Bot,e_Z_Bot,f_Z_Bot,g_Z_Bot,h_Z_Bot)
		#print(Znext)
		
		angleX=posdict["X_1"]-posdict["X_2"]
		
		angleY=posdict["Y_1"]-posdict["Y_2"]
		
		
		
		sleepytime=0.01
		angle=math.pi-math.atan2(angleY,angleX)
		print(angle)
		
		quat = Rotation.from_euler("xyz", [-math.pi, 0, angle]).as_quat() #changes rotation to dexnet angle
		
		m6 = CartesianMotion(Affine([0.5, 0.0, 0.4],quat), ReferenceType.Absolute) #move to reference position
		robot.move(m6)
		#time.sleep(sleepytime)
		m6 = CartesianMotion(Affine([Xnext, Ynext, 0.3],quat), ReferenceType.Absolute) #moves above item 30cm
		robot.move(m6)
		#time.sleep(sleepytime)
		m6 = CartesianMotion(Affine([Xnext, Ynext, Znext],quat), ReferenceType.Absolute) #moves to grasp position
		
		robot.move(m6)
		#time.sleep(sleepytime)
		
		gripper.clamp() #clamps until force threshold
		
		quat = Rotation.from_euler("xyz", [-math.pi, 0, 0]).as_quat() #changes rotation to front
		
		#m6 = CartesianMotion(Affine([Xnext, Ynext, Znext],quat), ReferenceType.Absolute)
		#robot.move(m6)
		#time.sleep(sleepytime)
		m6 = CartesianMotion(Affine([0.5, 0.0, 0.4],quat), ReferenceType.Absolute) #move to reference position
		robot.move(m6)
		#time.sleep(sleepytime)
		m6 = CartesianMotion(Affine([0.1, -0.6, 0.4],quat), ReferenceType.Absolute) #move to box
		robot.move(m6)
		#time.sleep(sleepytime)
		
		gripper.move_async(0.08) #releases object
		
		
		
		time.sleep(0.1)
	
	
