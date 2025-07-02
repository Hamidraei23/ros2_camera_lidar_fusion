#!/usr/bin/env python3
import sys
import select
import termios
import tty
import threading
import socket
import time
import json
import signal

class KeyboardUDP:
    def __init__(self, udp_ip='192.168.199.198', udp_port=9999):
        # UDP target
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Motion parameters
        self.linear_speed = 0.0       # current linear speed
        self.angular_speed = 0.0      # current angular speed
        self.accel_step = 0.02        # increment per key press (m/s)
        self.turn_step = 0.2          # increment per key press (rad/s)
        self.max_linear = 0.5         # max linear speed
        self.max_angular = 1.0        # max angular speed

        self._running = True

        print(
            f"Keyboard → UDP Teleop started. Sending to {self.udp_ip}:{self.udp_port}\n"
            "  W/S: accelerate forward/backward\n"
            "  A/D: turn right/left\n"
            "  X: stop immediately\n"
            "  CTRL+C: quit"
        )

        # launch threads
        threading.Thread(target=self.keyboard_loop, daemon=True).start()
        threading.Thread(target=self.send_loop, daemon=True).start()

    def keyboard_loop(self):
        """Read one character at a time and adjust speeds."""
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setraw(fd)
        try:
            while self._running:
                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1).lower()

                    if key == 'w':
                        self.linear_speed = min(self.linear_speed + self.accel_step,
                                                self.max_linear)
                    elif key == 's':
                        self.linear_speed = max(self.linear_speed - self.accel_step,
                                                -self.max_linear)
                    elif key == 'a':
                        self.angular_speed = max(self.angular_speed - self.turn_step,
                                                 -self.max_angular)
                    elif key == 'd':
                        self.angular_speed = min(self.angular_speed + self.turn_step,
                                                 self.max_angular)
                    elif key == 'x':
                        self.linear_speed = 0.0
                        self.angular_speed = 0.0
                    elif key == '\x03':  # Ctrl-C
                        self._running = False
                        break
                    else:
                        continue

                    print(f"→ linear: {self.linear_speed:.2f} m/s, "
                          f"angular: {self.angular_speed:.2f} rad/s")
                time.sleep(0.01)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def send_loop(self):
        """At 10 Hz send the current speeds over UDP."""
        rate = 0.1  # seconds
        while self._running:
            msg = json.dumps({
                'linear_x': self.linear_speed,
                'angular_z': self.angular_speed
            })
            self.sock.sendto(msg.encode('utf-8'),
                             (self.udp_ip, self.udp_port))
            time.sleep(rate)

    def stop(self):
        self._running = False

def main():
    # allow overriding via command-line args
    ip = sys.argv[1] if len(sys.argv) > 1 else '192.168.199.198'
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 9999

    teleop = KeyboardUDP(udp_ip=ip, udp_port=port)

    # catch SIGINT cleanly
    def handler(sig, frame):
        teleop.stop()
        print("\nKeyboard → UDP Teleop stopped.")
        sys.exit(0)
    signal.signal(signal.SIGINT, handler)

    # just wait until stopped
    while teleop._running:
        time.sleep(1)

if __name__ == '__main__':
    main()
