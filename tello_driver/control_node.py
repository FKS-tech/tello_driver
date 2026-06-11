#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist

from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Joy


import sys
import termios
import tty
import select
import atexit
import socket

#========================================================#
#---------- MAPEAMENTO DE TECLAS PARA COMANDOS ----------#
#========================================================#

key_mapping = {
    'w': (0, 0, 100, 0),   #up
    's': (0, 0, -100, 0),  #down
    'a': (0, 0, 0, 100),   #rotate left
    'd': (0, 0, 0, -100),  #rotate right

    'j': (-100, 0, 0, 0),   #forward
    'l': (100, 0, 0, 0),  #back
    'i': (0, 100, 0, 0),   #left
    'k': (0, -100, 0, 0),  #right

    't': 'takeoff',
    'g': 'land'
}


#=======================================================#
#---------- INTERFACE DE COMANDO PARA O DRONE ----------#
#=======================================================#

class CommandInterface:
    """Cliente UDP simples para comandos diretos do SDK do Tello."""

    def __init__(self, ip: str = "192.168.10.1", port: int = 8889):
        """Abre socket UDP e ativa o modo SDK."""
        self.ip = ip
        self.port = port
        self.address = (self.ip, self.port)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('', 9001))
        self.socket.settimeout(5.0)

        self.activate_sdk()

    def activate_sdk(self):
        """Envia o comando `command` para entrar no modo SDK."""
        return self.send_command("command")
    
    def send_command(self, command: str):
        """Envia um comando bloqueante e retorna a resposta textual."""
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
        """Solicita decolagem pelo SDK."""
        return self.send_command("takeoff")
    
    def land(self):
        """Solicita pouso pelo SDK."""
        return self.send_command("land")
    
    def move(self, a, b, c, d):
        """Envia comando RC continuo, limitando cada eixo em -100..100."""
        def clamp(x):
            """Garante que o SDK receba cada eixo no intervalo permitido."""
            return max(-100, min(100, int(x)))
         
        command = f"rc {clamp(a)} {clamp(b)} {clamp(c)} {clamp(d)}"
        self.socket.sendto(command.encode(), self.address)

    def foward(self, distance: int):
        """Move para frente por distancia em centimetros."""
        return self.send_command(f"forward {distance}")
    
    def back(self, distance: int):
        """Move para tras por distancia em centimetros."""
        return self.send_command(f"back {distance}")
    
    def left(self, distance: int):
        """Move para esquerda por distancia em centimetros."""
        return self.send_command(f"left {distance}")
    
    def right(self, distance: int):
        """Move para direita por distancia em centimetros."""
        return self.send_command(f"right {distance}")
    
    def up(self, distance: int):
        """Sobe por distancia em centimetros."""
        return self.send_command(f"up {distance}")
    
    def down(self, distance: int):
        """Desce por distancia em centimetros."""
        return self.send_command(f"down {distance}")
    
    def cw(self, angle: int):
        """Gira no sentido horario em graus."""
        return self.send_command(f"cw {angle}")
    
    def ccw(self, angle: int):
        """Gira no sentido anti-horario em graus."""
        return self.send_command(f"ccw {angle}")

#====================================================#
#---------- Classe para leitura de teclado ----------#
#====================================================#

class KeyboardInput:
    """Le teclas do terminal sem exigir Enter entre comandos."""

    def __init__(self):
        """Salva configuracao do terminal quando stdin e TTY."""
        if sys.stdin.isatty():
            self.settings = termios.tcgetattr(sys.stdin)
            self.is_tty = True
            atexit.register(self.restore_terminal)
        else:
            self.is_tty = False

    def get_key(self):
        """Retorna uma tecla pressionada ou string vazia se nao houver tecla."""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def restore_terminal(self):
        """Restaura configuracao original do terminal."""
        if self.is_tty:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

#====================================================#
#---------- NÓ ROS2 PARA CONTROLE DO DRONE ----------#
#====================================================#


class ControlNode(Node):
    """No manual legado para controle por teclado e servicos Trigger."""

    def __init__(self):
        """Cria interface SDK direta, servicos takeoff/land e timer de teclado."""
        super().__init__('control_node')
        self.command_interface = CommandInterface()

        self.create_service(Trigger, 'takeoff', self.takeoff_callback)
        self.create_service(Trigger, 'land', self.land_callback)

        #self.cmd_joy_subscriber = self.create_subscription(Joy, 'joy', self.cmd_joy_callback, 10)

        self.keyboard = KeyboardInput()
        if not self.keyboard.is_tty:
            self.get_logger().warning("Não está rodando em modo TTY. Teclado não funcionará.")
            return
        
        self.create_timer(0.05, self.read_keyboard)
        


    def takeoff_callback(self, request, response):
        """Servico ROS que encaminha takeoff ao SDK."""
        result = self.command_interface.takeoff()
        response.success = (result == "ok")
        response.message = result
        return response

    def land_callback(self, request, response):
        """Servico ROS que encaminha land ao SDK."""
        result = self.command_interface.land()
        response.success = (result == "ok")
        response.message = result
        return response


    def read_keyboard(self):
        """Le teclado e converte teclas mapeadas em comandos RC."""
        key = self.keyboard.get_key()
        if key == '\x03':  # CTRL+C
            rclpy.shutdown()
        elif key in key_mapping:
            command = key_mapping[key]
            if command == 'takeoff':
                self.command_interface.takeoff()
            elif command == 'land':
                self.command_interface.land()
            else:
                self.command_interface.move(*command)    
        else:
            self.command_interface.move(0, 0, 0, 0)  # stop if no key is pressed       

    
def main(args=None):
    """Start the legacy keyboard/service control node."""
    rclpy.init(args=args)
    control_node = ControlNode()
    executor = MultiThreadedExecutor()
    executor.add_node(control_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
