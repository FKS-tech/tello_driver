#!/usr/bin/env python3

import socket
from typing import Optional


class TelloClient:
    def __init__(
        self,
        tello_ip: str = '192.168.10.1',
        tello_port: int = 8889,
        local_port: int = 9000,
        timeout: float = 5.0,
    ):
        self.tello_ip = tello_ip
        self.tello_port = tello_port
        self.address = (self.tello_ip, self.tello_port)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('', local_port))
        self.socket.settimeout(timeout)

    def close(self) -> None:
        try:
            self.socket.close()
        except Exception:
            pass

    def send_command(self, command: str) -> Optional[str]:
        try:
            self.socket.sendto(command.encode('utf-8'), self.address)
            response, _ = self.socket.recvfrom(1024)
            return response.decode('utf-8', errors='ignore').strip()
        except Exception:
            return None

    def send_rc(self, left_right: int, forward_back: int, up_down: int, yaw: int) -> bool:
        def clamp(value: float) -> int:
            return max(-100, min(100, int(value)))

        command = (
            f'rc {clamp(left_right)} {clamp(forward_back)} '
            f'{clamp(up_down)} {clamp(yaw)}'
        )

        try:
            self.socket.sendto(command.encode('utf-8'), self.address)
            return True
        except Exception:
            return False

    def enter_sdk_mode(self) -> Optional[str]:
        return self.send_command('command')

    def takeoff(self) -> Optional[str]:
        return self.send_command('takeoff')

    def land(self) -> Optional[str]:
        return self.send_command('land')

    def stream_on(self) -> Optional[str]:
        return self.send_command('streamon')

    def stream_off(self) -> Optional[str]:
        return self.send_command('streamoff')

    def emergency(self) -> Optional[str]:
        return self.send_command('emergency')