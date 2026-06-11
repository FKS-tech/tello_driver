#!/usr/bin/env python3

import socket
from typing import Optional


class TelloClient:
    """Small UDP client for the Tello SDK command channel.

    This helper deliberately stays independent from ROS. Nodes can reuse it to
    send SDK commands while tests can replace or exercise it without spinning a
    ROS executor.
    """

    def __init__(
        self,
        tello_ip: str = '192.168.10.1',
        tello_port: int = 8889,
        local_port: int = 9000,
        timeout: float = 5.0,
    ):
        """Open the UDP socket used for SDK commands and responses."""
        self.tello_ip = tello_ip
        self.tello_port = tello_port
        self.local_port = local_port
        self.timeout = timeout
        self.address = (self.tello_ip, self.tello_port)
        self._closed = False

        # Keep this class independent from ROS so it can be reused by nodes,
        # tests, and future autonomy helpers.
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('', local_port))
        self.socket.settimeout(timeout)

    def close(self) -> None:
        """Close the UDP socket safely, even if called more than once."""
        if self._closed:
            return

        try:
            self.socket.close()
        except Exception:
            pass
        finally:
            self._closed = True

    def send_command(self, command: str) -> Optional[str]:
        """Send a blocking SDK command and return the text response.

        Returns None when the socket is closed, the command times out, or the
        drone does not provide a readable response.
        """
        if self._closed:
            return None

        try:
            self.socket.sendto(command.encode('utf-8'), self.address)
            response, _ = self.socket.recvfrom(1024)
            return response.decode('utf-8', errors='ignore').strip()
        except Exception:
            return None

    def send_rc(self, left_right: int, forward_back: int, up_down: int, yaw: int) -> bool:
        """Send a non-blocking RC velocity command.

        The Tello SDK expects each axis in the range -100..100. This method
        clamps the values before sending and returns True only if the UDP packet
        was handed to the socket successfully.
        """
        if self._closed:
            return False

        def clamp(value: float) -> int:
            """Convert one RC axis to the integer range accepted by the SDK."""
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
        """Put the drone in SDK mode and return its response."""
        return self.send_command('command')

    def takeoff(self) -> Optional[str]:
        """Ask the drone to take off and return its response."""
        return self.send_command('takeoff')

    def land(self) -> Optional[str]:
        """Ask the drone to land and return its response."""
        return self.send_command('land')

    def stream_on(self) -> Optional[str]:
        """Enable the Tello video stream and return its response."""
        return self.send_command('streamon')

    def stream_off(self) -> Optional[str]:
        """Disable the Tello video stream and return its response."""
        return self.send_command('streamoff')

    def emergency(self) -> Optional[str]:
        """Trigger the emergency motor stop command and return its response."""
        return self.send_command('emergency')
