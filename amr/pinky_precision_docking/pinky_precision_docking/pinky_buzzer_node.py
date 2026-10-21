#!/usr/bin/env python3
"""
Pinky Pro Hardware Buzzer Node (ROS 2).
Uses official 'pinkylib.Buzzer' to produce hardware melodies and reverse beeps.

Subscribed Topics:
  - /pinky/buzzer/command (std_msgs/msg/String):
      * 'docking_complete' : Plays '도-미-솔-도' (C5-E5-G5-C6)
      * 'reverse_start'     : Starts industrial backup beep ('삐~ 삐~', 1000Hz loop)
      * 'reverse_stop'      : Stops backup beep
"""

import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Try importing official pinkylib on Pinky Pro hardware
try:
    from pinkylib import Buzzer
    HAS_PINKYLIB = True
except ImportError:
    HAS_PINKYLIB = False
    Buzzer = None


class PinkyBuzzerNode(Node):
    def __init__(self):
        super().__init__('pinky_buzzer_node')

        self.buzzer = None
        if HAS_PINKYLIB:
            try:
                self.buzzer = Buzzer()
                self.buzzer.buzzer_start()
                self.get_logger().info('Successfully initialized official Pinky Pro Buzzer via pinkylib.')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize pinkylib.Buzzer: {e}')
        else:
            self.get_logger().warn('pinkylib not installed in this environment (Simulation / PC mode).')

        self._reverse_active = False
        self._reverse_thread = None
        self._lock = threading.Lock()

        self.sub_cmd = self.create_subscription(
            String,
            '/pinky/buzzer/command',
            self._command_cb,
            10
        )
        self.get_logger().info('PinkyBuzzerNode ready. Listening on /pinky/buzzer/command...')

    def _command_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        self.get_logger().info(f"Received buzzer command: '{cmd}'")

        if cmd == 'docking_complete':
            self._stop_reverse()
            threading.Thread(target=self._play_docking_chime, daemon=True).start()

        elif cmd == 'reverse_start':
            self._start_reverse()

        elif cmd == 'reverse_stop':
            self._stop_reverse()

        else:
            self.get_logger().warn(f"Unknown buzzer command: '{cmd}'")

    def _play_docking_chime(self):
        """Play '도-미-솔-도' (C5, E5, G5, C6) ascending melody."""
        self.get_logger().info("🎶 [Hardware Buzzer] Playing '도-미-솔-도' Docking Complete Chime...")
        notes = [
            (523, 0.12),  # C5 (도)
            (659, 0.12),  # E5 (미)
            (784, 0.12),  # G5 (솔)
            (1046, 0.15), # C6 (높은 도, 깔끔하게 마무리)
        ]

        if self.buzzer:
            try:
                with self._lock:
                    for freq, dur in notes:
                        self.buzzer.set_buzzer_freq(int(freq))
                        self.buzzer.set_buzzer_duty(50)
                        time.sleep(dur)
                        self.buzzer.set_buzzer_duty(0)
                        time.sleep(0.02)
            except Exception as e:
                self.get_logger().error(f'Error playing chime: {e}')
        else:
            # Terminal bell fallback if no hardware
            sys.stdout.write('\a')
            sys.stdout.flush()

    def _start_reverse(self):
        """Start industrial backup beep ('삐~ 삐~', 1000Hz)."""
        with self._lock:
            if self._reverse_active:
                return
            self._reverse_active = True
            self._reverse_thread = threading.Thread(target=self._reverse_loop, daemon=True)
            self._reverse_thread.start()

    def _reverse_loop(self):
        self.get_logger().info("🔊 [Hardware Buzzer] '삐~ 삐~' Undocking Reverse Beep Started...")
        while self._reverse_active:
            if self.buzzer:
                try:
                    self.buzzer.set_buzzer_freq(1000)
                    self.buzzer.set_buzzer_duty(50)
                    time.sleep(0.25)
                    self.buzzer.set_buzzer_duty(0)
                    time.sleep(0.20)
                except Exception as e:
                    self.get_logger().error(f'Error during reverse beep: {e}')
                    break
            else:
                time.sleep(0.45)

    def _stop_reverse(self):
        with self._lock:
            self._reverse_active = False
            if self.buzzer:
                try:
                    self.buzzer.set_buzzer_duty(0)
                except Exception:
                    pass
        self.get_logger().info("🛑 [Hardware Buzzer] Reverse Beep Stopped.")

    def destroy_node(self):
        self._stop_reverse()
        if self.buzzer:
            try:
                self.buzzer.set_buzzer_duty(0)
                self.buzzer.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PinkyBuzzerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
