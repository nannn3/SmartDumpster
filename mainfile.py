import math
import time
import ast
import numpy as np
from scipy.spatial.transform import Rotation
from franky import JointWaypointMotion, JointWaypoint, JointPositionStopMotion, CartesianMotion, CartesianWaypointMotion, CartesianWaypoint, Affine, RobotPose, ReferenceType, CartesianPoseStopMotion
from franky import Robot
from franky import Gripper

# Constants
CALIBRATION_FILE_PATH = "items/Calibration.txt"
START_FILE_PATH = "items/START.txt"
FINISHED_FILE_PATH = "items/Finished_Items.txt"
PENDING_FILE_PATH = "items/Items_Rot_Dep.txt"
ROBOT_IP = "172.16.0.2"
GRIPPER_SPEED = 0.3
GRIPPER_FORCE = 50
DROP_BOX = (0.55, -0.6, 0.35)
REST_POS = (0.5, 0.0, 0.4)

def load_calibration(file_path):
    """
    Load calibration data from a file.

    Args:
        file_path (str): Path to the calibration file.

    Returns:
        dict: Calibration data.
    """
    with open(file_path, "r") as calibration_file:
        current_calibration = calibration_file.read()
    nline = 0
    numcal = 0
    bigdict = {}
    while numcal < 8:
        numcal += 1
        endofline = current_calibration.find('\n', nline + 1)
        caldict = ast.literal_eval(current_calibration[nline:endofline])
        bigdict[caldict["Name"]] = caldict
        nline = endofline
    return bigdict

def get_camera_coordinates(calibration_data, name):
    """
    Retrieve camera coordinates from calibration data.

    Args:
        calibration_data (dict): Calibration data.
        name (str): Name of the calibration.

    Returns:
        tuple: Camera coordinates (X, Y, Depth).
    """
    curcal = calibration_data[name]
    return (curcal["X_0"], curcal["Y_0"], curcal["Dep"])

def solve_coefficients(cam_coordinates, bot_coordinates):
    """
    Solve for transformation coefficients between camera and bot coordinates.

    Args:
        cam_coordinates (dict): Dictionary of camera coordinates.
        bot_coordinates (dict): Dictionary of bot coordinates.

    Returns:
        tuple: Coefficients for X, Y, and Z transformations.
    """
    if cam_coordinates.keys() != bot_coordinates.keys():
        raise ValueError("Camera coordinates and bot coordinates keys do not match")

    keys = cam_coordinates.keys()
    cam_in = np.array([[x*y*z, x * x, y * y, z * z, x, y, z, 1] for key in keys for x, y, z in [cam_coordinates[key]]])
    bot_out_x = np.array([bot_coordinates[key][0] for key in keys])
    bot_out_y = np.array([bot_coordinates[key][1] for key in keys])
    bot_out_z = np.array([bot_coordinates[key][2] for key in keys])

    a_X_Bot, b_X_Bot, c_X_Bot, d_X_Bot, e_X_Bot, f_X_Bot, g_X_Bot, h_X_Bot = np.linalg.solve(cam_in, bot_out_x)
    a_Y_Bot, b_Y_Bot, c_Y_Bot, d_Y_Bot, e_Y_Bot, f_Y_Bot, g_Y_Bot, h_Y_Bot = np.linalg.solve(cam_in, bot_out_y)
    a_Z_Bot, b_Z_Bot, c_Z_Bot, d_Z_Bot, e_Z_Bot, f_Z_Bot, g_Z_Bot, h_Z_Bot = np.linalg.solve(cam_in, bot_out_z)
    print((a_X_Bot, b_X_Bot, c_X_Bot, d_X_Bot, e_X_Bot, f_X_Bot, g_X_Bot, h_X_Bot), "\n",
           (a_Y_Bot, b_Y_Bot, c_Y_Bot, d_Y_Bot, e_Y_Bot, f_Y_Bot, g_Y_Bot, h_Y_Bot), "\n",
           (a_Z_Bot, b_Z_Bot, c_Z_Bot, d_Z_Bot, e_Z_Bot, f_Z_Bot, g_Z_Bot, h_Z_Bot))	
    #Xcam=349
    #Ycam=165
    #Zdep=1.0833794
    
    Xcam=330
    Ycam=87
    Zdep=1.106293
    Xnext = (a_X_Bot * Xcam * Ycam * Zdep + b_X_Bot * Xcam * Xcam + c_X_Bot * Ycam * Ycam + 
		 d_X_Bot * Zdep * Zdep + e_X_Bot * Xcam + f_X_Bot * Ycam + g_X_Bot * Zdep + h_X_Bot)
    Ynext = (a_Y_Bot * Xcam * Ycam * Zdep + b_Y_Bot * Xcam * Xcam + c_Y_Bot * Ycam * Ycam + 
		 d_Y_Bot * Zdep * Zdep + e_Y_Bot * Xcam + f_Y_Bot * Ycam + g_Y_Bot * Zdep + h_Y_Bot)
    Znext = (a_Z_Bot * Xcam * Ycam * Zdep + b_Z_Bot * Xcam * Xcam + c_Z_Bot * Ycam * Ycam + 
		 d_Z_Bot * Zdep * Zdep + e_Z_Bot * Xcam + f_Z_Bot * Ycam + g_Z_Bot * Zdep + h_Z_Bot)
    print(Xnext, Ynext, Znext)
    return (a_X_Bot, b_X_Bot, c_X_Bot, d_X_Bot, e_X_Bot, f_X_Bot, g_X_Bot, h_X_Bot), \
           (a_Y_Bot, b_Y_Bot, c_Y_Bot, d_Y_Bot, e_Y_Bot, f_Y_Bot, g_Y_Bot, h_Y_Bot), \
           (a_Z_Bot, b_Z_Bot, c_Z_Bot, d_Z_Bot, e_Z_Bot, f_Z_Bot, g_Z_Bot, h_Z_Bot)

def initialize_robot(ip, speed, force):
    """
    Initialize the robot and gripper.

    Args:
        ip (str): IP address of the robot.
        speed (float): Speed threshold for the gripper.
        force (float): Force threshold for the gripper.

    Returns:
        tuple: Initialized robot and gripper objects.
    """
    robot = Robot(ip)
    gripper = Gripper(ip, speed, force)
    robot.recover_from_errors()
    robot.relative_dynamics_factor = 0.3
    gripper.move_async(0.08)
    return robot, gripper

def move_robot_to_position(robot, position, quaternion, reference_type=ReferenceType.Absolute):
    """
    Move the robot to a specified position.

    Args:
        robot (Robot): The robot object.
        position (tuple): Target position (x, y, z).
        quaternion (tuple): Target orientation as a quaternion.
        reference_type (ReferenceType): Reference type for the movement.
    """
    motion = CartesianMotion(Affine(position, quaternion), reference_type)
    robot.move(motion)

def process_pending_items(robot, gripper, coefficients, start_file, pending_file, finished_file):
    """
    Process items listed in the pending file.

    Args:
        robot (Robot): The robot object.
        gripper (Gripper): The gripper object.
        coefficients (tuple): Transformation coefficients for X, Y, and Z.
        start_file (str): Path to the start file.
        pending_file (str): Path to the pending items file.
        finished_file (str): Path to the finished items file.
    """
    a_X_Bot, b_X_Bot, c_X_Bot, d_X_Bot, e_X_Bot, f_X_Bot, g_X_Bot, h_X_Bot = coefficients[0]
    a_Y_Bot, b_Y_Bot, c_Y_Bot, d_Y_Bot, e_Y_Bot, f_Y_Bot, g_Y_Bot, h_Y_Bot = coefficients[1]
    a_Z_Bot, b_Z_Bot, c_Z_Bot, d_Z_Bot, e_Z_Bot, f_Z_Bot, g_Z_Bot, h_Z_Bot = coefficients[2]

    while True:
        with open(start_file, "r") as file:
            if len(file.read()) > 2:
                with open(start_file, "w") as file_write:
                    file_write.write("")

                with open(pending_file, "r") as pending:
                    current_pending = pending.read()

                with open(pending_file, "w") as pending_write:
                    pending_write.write("")

                nextline = 0

                if len(current_pending) > 2:
                    while len(current_pending) > nextline:
                        end_of_line = current_pending.find('\n', nextline)
                        posdict = ast.literal_eval(current_pending[nextline:end_of_line])

                        Xcam = posdict["X_0"]
                        Ycam = posdict["Y_0"]
                        Zdep = posdict["Dep"]
                        nextline = end_of_line + 1

                        Xnext = (a_X_Bot * Xcam * Ycam * Zdep + b_X_Bot * Xcam * Xcam + c_X_Bot * Ycam * Ycam + 
				 d_X_Bot * Zdep * Zdep + e_X_Bot * Xcam + f_X_Bot * Ycam + g_X_Bot * Zdep + h_X_Bot)
                        Ynext = (a_Y_Bot * Xcam * Ycam * Zdep + b_Y_Bot * Xcam * Xcam + c_Y_Bot * Ycam * Ycam + 
				 d_Y_Bot * Zdep * Zdep + e_Y_Bot * Xcam + f_Y_Bot * Ycam + g_Y_Bot * Zdep + h_Y_Bot)
                        Znext = (a_Z_Bot * Xcam * Ycam * Zdep + b_Z_Bot * Xcam * Xcam + c_Z_Bot * Ycam * Ycam + 
				 d_Z_Bot * Zdep * Zdep + e_Z_Bot * Xcam + f_Z_Bot * Ycam + g_Z_Bot * Zdep + h_Z_Bot)
                        print(Xnext)
                        print(Ynext)
                        print(Znext)

                        angleX = posdict["X_1"] - posdict["X_2"]
                        angleY = posdict["Y_1"] - posdict["Y_2"]
                        angle = math.pi - math.atan2(angleY, angleX)

                        quat = Rotation.from_euler("xyz", [-math.pi, 0, angle]).as_quat()

                        move_robot_to_position(robot, REST_POS, quat)
                        robot.relative_dynamics_factor = 0.3
                        move_robot_to_position(robot, (Xnext, Ynext, 0.3), quat)
                        move_robot_to_position(robot, (Xnext, Ynext, Znext), quat)

                        gripper.clamp()
                        current_pose = robot.current_pose
                        print(current_pose)

                        quat = Rotation.from_euler("xyz", [-math.pi, 0, 0]).as_quat()

                        move_robot_to_position(robot, REST_POS, quat)
                        robot.relative_dynamics_factor = 0.3
                        quat = Rotation.from_euler("xyz", [-math.pi, -math.pi/4, -math.pi/2]).as_quat()
                        
                        move_robot_to_position(robot, DROP_BOX, quat)

                        gripper.move_async(0.08)
                        time.sleep(0.1)

def main():
    # Load calibration data
    calibration_data = load_calibration(CALIBRATION_FILE_PATH)

    # Get camera coordinates
    cam_coordinates = {
        "Short_White": get_camera_coordinates(calibration_data, "Short_White"),
        "Short_Blue": get_camera_coordinates(calibration_data, "Short_Blue"),
        "Short_Green": get_camera_coordinates(calibration_data, "Short_Green"),
        "Short_Orange": get_camera_coordinates(calibration_data, "Short_Orange"),
        "Tall_Green": get_camera_coordinates(calibration_data, "Tall_Green"),
        "Tall_Orange": get_camera_coordinates(calibration_data, "Tall_Orange"),
        "Tall_White": get_camera_coordinates(calibration_data, "Tall_White"),
        "Tall_Blue": get_camera_coordinates(calibration_data, "Tall_Blue")
    }

    # Define bot coordinates
    bot_coordinates = {
        "Short_White": (0.3, 0.15, 0.21),
        "Short_Blue": (0.3, -0.05, 0.21),
        "Short_Green": (0.76, -0.15, 0.21),
        "Short_Orange": (0.76, 0.05, 0.21),
        "Tall_Green": (0.3, -0.15, 0.26),
        "Tall_Orange": (0.3, 0.05, 0.26),
        "Tall_White": (0.76, 0.15, 0.26),
        "Tall_Blue": (0.76, -0.05, 0.26)
    }

    # Solve for coefficients
    coefficients = solve_coefficients(cam_coordinates, bot_coordinates)

    # Initialize robot and gripper
    robot, gripper = initialize_robot(ROBOT_IP, GRIPPER_SPEED, GRIPPER_FORCE)

    # Move robot to start position
    quat = Rotation.from_euler("xyz", [-math.pi, 0,0]).as_quat()
    move_robot_to_position(robot, (0.5, 0.0, 0.6), quat)

    # Process pending items
    process_pending_items(robot, gripper, coefficients, START_FILE_PATH, PENDING_FILE_PATH, FINISHED_FILE_PATH)

if __name__ == "__main__":
    main()
