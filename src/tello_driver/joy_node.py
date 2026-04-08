#!/usr/bin/env python3

from urllib import response
import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Joy

import socket
import time

#================================================================#
#---------- MAPEAMENTO DE BOTÕES E EIXOS PARA COMANDOS ----------#
#================================================================#

#eixos
rotation= 0
heght = 1
forward_back = 4
left_right = 3
#buttons
land_button = 0
takeoff_button = 3

#=======================================================#
#---------- INTERFACE DE COMANDO PARA O DRONE ----------#
#=======================================================#

class CommandInterface:
    def __init__(self, ip: str = "192.168.10.1", port: int = 8889):
        self.ip = ip
        self.port = port
        self.address = (self.ip, self.port)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('', 9000))
        self.socket.settimeout(5.0)

        

        self.activate_sdk()
        print("SDK:", self.activate_sdk())
        time.sleep(2)

        self.stream_off()
        time.sleep(1)

        self.stream_on()

    def activate_sdk(self):
        return self.send_command("command")
    
    def send_command(self, command: str):
        try:
            self.socket.sendto(command.encode(), self.address)
            response, _ = self.socket.recvfrom(1024)
            return response.decode('utf-8', errors='ignore')
        except Exception as e:
            print(f"Error sending command: {e}")
            return None
        
    #===================================================#
    #---------- COMANDOS BASICOS DE MOVIMENTO ----------#
    #===================================================#

    def takeoff(self):
        return self.send_command("takeoff")
    
    def land(self):
        return self.send_command("land")
    
    def move(self, a, b, c, d):
        def clamp(x):
            return max(-100, min(100, int(x)))
         
        command = f"rc {clamp(a)} {clamp(b)} {clamp(c)} {clamp(d)}"
        self.socket.sendto(command.encode(), self.address)

    def foward(self, distance: int):
        return self.send_command(f"forward {distance}")
    
    def back(self, distance: int):
        return self.send_command(f"back {distance}")
    
    def left(self, distance: int):
        return self.send_command(f"left {distance}")
    
    def right(self, distance: int):
        return self.send_command(f"right {distance}")
    
    def up(self, distance: int):
        return self.send_command(f"up {distance}")
    
    def down(self, distance: int):
        return self.send_command(f"down {distance}")
    
    def cw(self, angle: int):
        return self.send_command(f"cw {angle}")
    
    def ccw(self, angle: int):
        return self.send_command(f"ccw {angle}")
    
    #======================================================#
    #----------- COMANDOS DE DADOS DE STREAMING -----------#
    #======================================================#
    
    
    def stream_on(self):
        return self.send_command("streamon")
    
    def stream_off(self):
        return self.send_command("streamoff")
    
    
    
class JoyNode(Node):
    def __init__(self):
        super().__init__('joy_node')
        self.command_interface = CommandInterface()

        self.cmd_joy_subscriber = self.create_subscription(Joy, 'joy', self.cmd_joy_callback, 10)
        
        

        self.current_cmd = (0, 0, 0, 0)

        self.current_cmd_timer = self.create_timer(0.05, self.send_current_cmd)

        self.last_takeoff = False
        self.last_land = False



    


    def cmd_joy_callback(self, msg):
        
        def dz(v): return 0.0 if abs(v) < 0.3 else v

        lr = dz(msg.axes[left_right]) * -100
        fb = dz(msg.axes[forward_back]) * 100
        ht = dz(msg.axes[heght]) * 100
        rt = dz(msg.axes[rotation]) * -100

        if msg.buttons[takeoff_button] and not self.last_takeoff:
            self.command_interface.takeoff()
            self.get_logger().warn("Tello esta decolando")
            self.last_takeoff = True

        elif not msg.buttons[takeoff_button]:
            self.last_takeoff = False


        if msg.buttons[land_button] and not self.last_land:
            self.command_interface.land()
            self.get_logger().warn("Tello esta aterrissando")
            self.last_land = True

        elif not msg.buttons[land_button]:
            self.last_land = False

        
        self.current_cmd = (lr, fb, ht, rt)

    def send_current_cmd(self):
        lr, fb, ht, rt = self.current_cmd
        self.command_interface.move(lr, fb, ht, rt)



    
def main(args=None):
    rclpy.init(args=args)
    joy_node = JoyNode()
    executor = MultiThreadedExecutor()
    executor.add_node(joy_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        joy_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()