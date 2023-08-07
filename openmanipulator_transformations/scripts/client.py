#!/usr/bin/env python3

import os
from threading import Thread

import rospy
from kinematics.config import ROBOT_CONTROL_SERVICE_NAME, INV_TRANSFORM_SERVICE_NAME, FWD_TRANSFORM_SERVICE_NAME, CLI_NODE_NAME
from kinematics.utils import unwrap_angles
from open_manipulator_msgs.msg import JointPosition
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest
from sensor_msgs.msg import JointState

from openmanipulator_transformations.srv import Transform, TransformRequest, TransformResponse


"""
Command-line interface for controlling the OpenManipulator-X robot arm by reading position commands 
in cartesian coordinates from user input, transforming them to robot parameters using the 
inverse-kinematics service and sending the paramters to the robot controller
"""


def control_robot(
    vec,
    path_time=3.0,
    max_accelerations_scaling_factor=1.0,
    max_velocity_scaling_factor=1.0,
    planning_group_name="robot matec",
):
    """Sends a request to move the robot joints to the required angles

    Args:
        vec: List of angles. Length 4
        path_time (float, optional): Movement duration. Defaults to 5.0.
        max_accelerations_scaling_factor (float, optional): Defaults to 1.0. Doesn't change much the robot behavior
        max_velocity_scaling_factor (float, optional): Defaults to 1.0. Doesn't change much the robot behavior
        planning_group_name (str, optional): For logs only. Defaults to "robot matec".

    Returns:
        Response
    """

    if len(vec) != 4:
        return

    # Send joint angles to the robot controller
    rospy.wait_for_service(ROBOT_CONTROL_SERVICE_NAME)

    try:
        controller_service = rospy.ServiceProxy(ROBOT_CONTROL_SERVICE_NAME, SetJointPosition)

        req = SetJointPositionRequest()
        req.planning_group = planning_group_name
        req.joint_position = JointPosition(
            ["joint1", "joint2", "joint3", "joint4"],
            vec,
            max_accelerations_scaling_factor,
            max_velocity_scaling_factor,
        )
        req.path_time = path_time

        res = controller_service(req)
        if res.is_planned:
            print("[+] Comunicación con el robot exitosa.")
        else:
            print("[-] El recorrido no pudo ser planeado.")

        return res
    except rospy.ServiceException as e:
        print(f"[!] Llamada al servicio fallida: {e}")


def input_position():
    # get position in space
    vec = []
    try:
        vec.append(float(input("Pos. X: ")))
        vec.append(float(input("Pos. Y: ")))
        vec.append(float(input("Pos. Z: ")))
        vec.append(float(input("Ángulo de agarre: ")))
        return vec
    except ValueError:
        print("[!] Error al ingresar valores.")
        return


def inverse_transform(vec):
    # Gets joint values from the transformation service
    rospy.wait_for_service(INV_TRANSFORM_SERVICE_NAME)

    try:
        transformation_service = rospy.ServiceProxy(INV_TRANSFORM_SERVICE_NAME, Transform)

        res = transformation_service(vec)  # Transform x, y, z position to joint angles

        vec_joint = res.output

        unwrap_angles(list(vec_joint))

        return vec_joint

    except rospy.ServiceException as e:
        print(f"[!] Llamada al servicio fallida: {e}")

def forward_transform(vec):
    #Gets position from forward service
    rospy.wait_for_service(FWD_TRANSFORM_SERVICE_NAME)
    try:
        transformation_service = rospy.ServiceProxy(FWD_TRANSFORM_SERVICE_NAME, Transform)

        result = transformation_service(vec).output

        return result
    except: 
        print(f"[!] Llamada al servicio fallida")
    
def get_xyz():
    try:
        states = rospy.wait_for_message("/joint_states", JointState, timeout=5)
        joint_angles = states.position[2:6]  # Get only states of joints 1 to 4

        print(f'Ángulos de las articulaciones (1-4): {", " .join([f"{x:.3f}" for x in joint_angles])}')
        result = forward_transform(joint_angles)
        print(f"Posición en los ejes coordenados (X, Y, Z): " + ", " .join([f"{x:.3f}" for x in result]))
    except rospy.ROSException:
        print("[E] Tiemout exceeded")


def show_menu():
    print(
        """

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

    Bienvenido al nodo Cliente. Envía la posición de las articulaciones a partir de la posición del end-effector deseada.

    Ingresa "?" para ver la lista de comandos y "exit" para salir

    """
    )


def show_help():
    print(
        """
    
    Comando                 Descripción

    help         ?          Muestra este menú
    setPos       position   Envio de posición al robot
    getPos       g          Solicitud de la posicion actual del robot en los ejes coordenados
    home                    Envio de posición original
    verbose      v          Activar o desactivar modo descriptivo 
    exit         q          Finalizar programa
    clc                     Limpiar consola

    Ejemplo de uso:
    > pos
        Pos. X: 1.0
        Pos. Y: 1.0
        Pos. Z: 1.0
        Ángulo de agarre (rad): 1.57

    Posicion X, Y y Z es la coordenada en el espacio
    Ángulo de agarre es la direccion de la pinza 

    Descripción de algunos ángulos de agarre desde la posicion origen:
        Apuntando hacia el cielo:  1.57 ( pi/2)
        Apuntando hacia el suelo: -1.57 (-pi/2)
        Apuntando hacia el exterior:  0
        
    """
    )


def clc():
    os.system("clear")

def main():
    clc()
    show_menu()
    verbose_mode = False

    while True:
        # Enter command
        command = input("\n> ").strip()

        # Filter commands
        if command == "setPos" or command == "position":
            # Enter position to send to the robot
            vec_position = input_position()

            if vec_position == None:
                print("[-] Error al ingresar posicion.")
                continue

            # Show information in description mode
            if verbose_mode:
                print(f"[*] Vector de posicion ingresado: {vec_position}")
                print("[*] Envio del vector de posiciones al servicio para transformar.")

            # Convert X Y Z t position to robot's angles
            vec_joint = inverse_transform(vec_position)

            if vec_joint == None:
                print("[-] Error al transformar posicion.")
                continue

            # Show information in description mode
            if verbose_mode:
                print("[*] Vector de angulos del motor recibido.")
                print(f"[*] Vector de posicion transformado: {vec_position}")
                print("[*] Envio de angulos al robot.")

            control_robot(vec_joint)

        elif command == "home":
            # Send init pose
            if verbose_mode:
                print("[*] Envio posicion origen.")
            control_robot([0, 0, 0, 0])

        elif command == "help" or command == "?":
            # Show menu and usage example
            show_help()

        elif command == "clc":
            # clean console
            clc()
            show_menu()

        elif command == "verbose" or command == "v":
            # change verbose mode
            verbose_mode = not verbose_mode
            if verbose_mode:
                print("[*] Modo descriptivo activado.")
            else:
                print("[*] Modo descriptivo desactivado.")

        elif command == "getPos" or command == "g":
            if verbose_mode:
                print("Obteniendo posición en X Y Z")
            get_xyz()

        elif command == "exit" or command == "q":
            # Finalize program
            break

        else:
            print("[!] Comando ingresado no reconocido")


if __name__ == "__main__":
    rospy.init_node(CLI_NODE_NAME)
    Thread(target=main()).start()
    rospy.spin()
