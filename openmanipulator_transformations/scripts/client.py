#!/usr/bin/env python3

import math
import sys
import os
import rospy
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest
from open_manipulator_msgs.msg import JointPosition
from openmanipulator_transformations.srv import Transform

"""
JointPosition.msg:
string[]   joint_name
float64[]  position
float64    max_accelerations_scaling_factor
float64    max_velocity_scaling_factor

SetJointPosition.srv:
string planning_group
JointPosition joint_position
float64 path_time
---
bool is_planned
"""

ROBOT_CONTROL_SERVICE_NAME = "/goal_joint_space_path"
TRANSFORM_SERVICE_NAME = "/transformations/transform"


def control_robot(vec):
    # Send to robot joint angles
    rospy.wait_for_service(ROBOT_CONTROL_SERVICE_NAME)
    try:
        service = rospy.ServiceProxy(ROBOT_CONTROL_SERVICE_NAME, SetJointPosition)
        
        # Group name for logs
        planning_group_name = "robot matec"
        
        # Some values that not change to much the robot's behavior
        max_accelerations_scaling_factor = 1.0
        max_velocity_scaling_factor = 1.0

        # Time from start to end movement
        path_time = 5.0

        req = SetJointPositionRequest()
        req.planning_group = planning_group_name
        req.joint_position  = JointPosition(["joint1", "joint2", "joint3", "joint4"], [vec[0], vec[1], vec[2], vec[3]], max_accelerations_scaling_factor, max_velocity_scaling_factor) 
        req.path_time = path_time

        resp = service(req)

        print("[+] Comunicación con el robot exitosa.")

        return resp
    except rospy.ServiceException as e:
        print(f"[!] Llamada al servicio fallida: {e}")


def input_position():
    # get position in space
    vec = []
    try:    
        vec.append(float(input("Pos. X: ")))
        vec.append(float(input("Pos. Y: ")))
        vec.append(float(input("Pos. Z: ")))
        return vec
    except ValueError:
        print("[!] Error al ingresar un valor.")
        return 

def unwrap_angles(vec):
    # Adjust positions within the range of -pi to pi for the robot's movement
    vec_list = list(vec)  # Convert tuple to list

    for i in range(3):
        while vec_list [i] > math.pi or vec_list [i] < -math.pi : 
            if vec_list [i] > math.pi:
                vec_list[i] -= 2*math.pi
            elif vec_list[i] < -math.pi:
                vec_list[i] += 2*math.pi

    vec_modified = tuple(vec_list)  # Convert list to tuple 
    return vec_modified


def transform_position(vec):
    rospy.wait_for_service(TRANSFORM_SERVICE_NAME)
    
    try:
        service = rospy.ServiceProxy(TRANSFORM_SERVICE_NAME, Transform)        
        
        resp = service(vec) # Transform x, y, z position to joint angles

        vec_joint = resp.joint_angles 

        unwrap_angles(vec_joint) 
        
        return vec_joint
        
    except rospy.ServiceException as e:
        print(f"[!] Llamada al servicio fallida: {e}") 


def show_menu(): 
    print("""

 .----------------.  .----------------.  .----------------.  .----------------.  .----------------.   
| .--------------. || .--------------. || .--------------. || .--------------. || .--------------. |  
| |  _______     | || |     ____     | || |   ______     | || |     ____     | || |  _________   | |  
| | |_   __ \    | || |   .'    `.   | || |  |_   _ \    | || |   .'    `.   | || | |  _   _  |  | |  
| |   | |__) |   | || |  /  .--.  \  | || |    | |_) |   | || |  /  .--.  \  | || | |_/ | | \_|  | |  
| |   |  __ /    | || |  | |    | |  | || |    |  __'.   | || |  | |    | |  | || |     | |      | |  
| |  _| |  \ \_  | || |  \  `--'  /  | || |   _| |__) |  | || |  \  `--'  /  | || |    _| |_     | |  
| | |____| |___| | || |   `.____.'   | || |  |_______/   | || |   `.____.'   | || |   |_____|    | |  
| |              | || |              | || |              | || |              | || |              | |  
| '--------------' || '--------------' || '--------------' || '--------------' || '--------------' |  
 '----------------'  '----------------'  '----------------'  '----------------'  '----------------'   
 .----------------.  .----------------.  .----------------.  .----------------.  .----------------.   
| .--------------. || .--------------. || .--------------. || .--------------. || .--------------. |  
| | ____    ____ | || |      __      | || |  _________   | || |  _________   | || |     ______   | |  
| ||_   \  /   _|| || |     /  \     | || | |  _   _  |  | || | |_   ___  |  | || |   .' ___  |  | |  
| |  |   \/   |  | || |    / /\ \    | || | |_/ | | \_|  | || |   | |_  \_|  | || |  / .'   \_|  | |  
| |  | |\  /| |  | || |   / ____ \   | || |     | |      | || |   |  _|  _   | || |  | |         | |  
| | _| |_\/_| |_ | || | _/ /    \ \_ | || |    _| |_     | || |  _| |___/ |  | || |  \ `.___.'\  | |  
| ||_____||_____|| || ||____|  |____|| || |   |_____|    | || | |_________|  | || |   `._____.'  | |  
| |              | || |              | || |              | || |              | || |              | |  
| '--------------' || '--------------' || '--------------' || '--------------' || '--------------' |  
 '----------------'  '----------------'  '----------------'  '----------------'  '----------------'   

    Bienvenido al nodo Cliente, envía una posición en los ejes x, y, z a las que desea mover el robot.

    Ingresa "?" para ver la lista de comandos

    """)

def show_commands():
    print("""
    
    Comando             Descripción

    help    ?           Muestra este menú
    pos     position    Envio de posición al robot
    home                Envio de posición original 
    exit    q           Finalizar programa
    clc                 Limpiar consola

    Ejemplo de uso:
    > pos
        Pos. X: 1.8
        Pos. Y: 0.5
        Pos. Z: -0.2

    """)

def clc():
    if os.name == 'nt':  # For Windows
        os.system('cls')
    else:  # For Unix (Linux, macOS)
        os.system('clear')

if __name__ == "__main__": 
    clc()
    show_menu()

    while True:
        # Enter command
        command = input("\n> ")
        
        # Filter commands
        if command == 'pos' or command == 'position':
            # Enter position to send to the robot
            vec_position = input_position()
            
            if vec_position != None:
                vec_joint = transform_position(vec_position)
                if vec_joint != None:
                    control_robot(vec_joint)
                else:
                    print("[-] Error al transformar posicion.")
            
        elif command == 'home':
            # Send init pose
            print("[*] Envio posicion origen.")
            control_robot([0, 0, 0, 0])
            
        elif command == 'help' or command == '?':
            # Show menu and usage example
            show_commands()
        
        elif command == 'clc':
            # clean console
            clc()
        
        elif command == 'exit' or command == 'q':
            # Finalize program
            break

        else: 
            print("[!] Comando ingresado no reconocido")
