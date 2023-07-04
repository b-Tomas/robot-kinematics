#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class TalkerNode(object):

    def __init__(self):
        rospy.init_node("talker_node")
        rospy.loginfo("[*] Talker Node Initialized")
        self.publisher_ = rospy.Publisher('data_transfer', String, queue_size=10)

    def talker(self):
        rospy.loginfo("\nIniciando Talker (CLI)")
        self.menu()

        while True:
            s = input("[?] Desea ingresar una posicion? [N/s/?]")

            if s.startswith("?"):
                self.menu()
                continue

            if not s.lower().startswith("s"):
                print("[-] Se desea terminar con el programa. :D")
                break

            x = input("Pos. X: ")
            y = input("Pos. Y: ")
            z = input("Pos. Z: ")
            print("\n")

            msg = String()
            msg.data = self.convert_position(x, y, z)

            self.publisher_.publish(msg)

            rospy.sleep(1.0)

    def menu(self):
        print("\n")
        print("Bienvenido al nodo Talker, envía una posición en los ejes x, y, z a un nodo en modo escucha.\n")
        print("Ejemplo de uso:")
        print("Pos. X: 22.3")
        print("Pos. Y: 43.8")
        print("Pos. Z: 12.4")
        print("\n\n")

    def convert_position(self, x, y, z):
        vector = [x, y, z, 0]
        return str(vector[0]) + "-" + str(vector[1]) + "-" + str(vector[2]) + "-" + str(vector[3])


def main():
    talker_node = TalkerNode()
    talker_node.talker()

if __name__ == "__main__":
    main()
