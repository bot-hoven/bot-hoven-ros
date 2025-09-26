import math
import sys
import threading
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from control_msgs.msg import MultiDOFCommand, MultiDOFStateStamped

@dataclass
class AxisConfig:
    controller: str
    dof: str
    amplitude: float
    freq_hz: float
    phase_deg: float = 0.0
    offset: float = 0.0
    center_at_current: bool = False

class SinePublisher(Node):
    def __init__(self, axes, rate_hz=20.0, duration=None, send_vel_dot=False):
        super().__init__("sine_pid_reference")
        self.axes = axes
        self.rate_hz = rate_hz
        self.duration = duration
        self.send_vel_dot = send_vel_dot
        self.start_time = None
        self.stop_flag = False

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # One publisher per controller
        self.pubs = {}
        for ax in self.axes:
            topic = f"/{ax.controller}/reference"
            if topic not in self.pubs:
                self.pubs[topic] = self.create_publisher(MultiDOFCommand, topic, qos)
                self.get_logger().info(f"Publishing references to {topic}")

        # Optionally center sines around current feedback
        self.pending_center = {}
        for ax in self.axes:
            if ax.center_at_current:
                topic = f"/{ax.controller}/controller_state"
                self.create_subscription(MultiDOFStateStamped, topic,
                                         lambda msg, ax=ax: self._on_state(ax, msg), qos)
                self.pending_center[(ax.controller, ax.dof)] = True
                self.get_logger().info(f"Waiting for initial state on {topic} to center {ax.dof}")

        self.timer = self.create_timer(1.0 / self.rate_hz, self._on_timer)

    def _on_state(self, ax: AxisConfig, msg: MultiDOFStateStamped):
        # Find matching dof and use its feedback as the offset center
        for s in msg.dof_states:
            if s.name == ax.dof:
                ax.offset = float(s.feedback)
                key = (ax.controller, ax.dof)
                if key in self.pending_center:
                    del self.pending_center[key]
                    self.get_logger().info(
                        f"Centered {ax.controller}/{ax.dof} at {ax.offset:.6f}"
                    )
                break

    def _all_centered(self):
        return len(self.pending_center) == 0

    def _on_timer(self):
        if self.start_time is None:
            if self.pending_center:
                # Wait until we get the first state(s) to center, but keep spinning
                return
            self.start_time = time.monotonic()

        t = time.monotonic() - self.start_time

        # Group commands by controller topic
        grouped = {}
        for ax in self.axes:
            # angle = 2π f t + phase
            ang = 2.0 * math.pi * ax.freq_hz * t + math.radians(ax.phase_deg)
            pos = ax.offset + ax.amplitude * math.sin(ang)
            if self.send_vel_dot:
                # derivative of position (rad/s * amplitude * cos)
                vel = ax.amplitude * (2.0 * math.pi * ax.freq_hz) * math.cos(ang)
            topic = f"/{ax.controller}/reference"
            if topic not in grouped:
                grouped[topic] = {"names": [], "vals": [], "vals_dot": []}
            grouped[topic]["names"].append(ax.dof)
            grouped[topic]["vals"].append(pos)
            if self.send_vel_dot:
                grouped[topic]["vals_dot"].append(vel)

        # Publish one MultiDOFCommand per controller
        for topic, payload in grouped.items():
            msg = MultiDOFCommand()
            msg.dof_names = payload["names"]
            msg.values = payload["vals"]
            if self.send_vel_dot:
                msg.values_dot = payload["vals_dot"]
            self.pubs[topic].publish(msg)

        if self.duration is not None and t >= self.duration:
            self.get_logger().info("Duration reached; stopping.")
            self.stop()

    def stop(self):
        self.stop_flag = True
        self.destroy_timer(self.timer)

def main():
    import argparse
    p = argparse.ArgumentParser(description="Publish sinusoidal position references to ros2_control PID controller(s).")
    p.add_argument("--rate", type=float, default=20.0, help="publish rate [Hz]")
    p.add_argument("--duration", type=float, default=None, help="stop after N seconds (omit to run until ENTER)")
    p.add_argument("--send-vel-dot", action="store_true",
                   help="also send values_dot (reference derivative). Only valid if controller is configured with ['position','velocity'].")
    # Left axis
    p.add_argument("--left", action="store_true", help="drive left_stepper_controller/lh_stepper")
    p.add_argument("--left-amp", type=float, default=0.1)
    p.add_argument("--left-freq", type=float, default=0.2)
    p.add_argument("--left-phase", type=float, default=0.0)
    p.add_argument("--left-offset", type=float, default=0.0)
    p.add_argument("--left-center", action="store_true", help="center left sine at current position")
    # Right axis
    p.add_argument("--right", action="store_true", help="drive right_stepper_controller/rh_stepper")
    p.add_argument("--right-amp", type=float, default=0.1)
    p.add_argument("--right-freq", type=float, default=0.2)
    p.add_argument("--right-phase", type=float, default=180.0)  # default out-of-phase
    p.add_argument("--right-offset", type=float, default=0.0)
    p.add_argument("--right-center", action="store_true", help="center right sine at current position")
    args = p.parse_args()

    axes = []
    if args.left:
        axes.append(AxisConfig("left_stepper_controller", "lh_stepper",
                               args.left_amp, args.left_freq, args.left_phase,
                               args.left_offset, args.left_center))
    if args.right:
        axes.append(AxisConfig("right_stepper_controller", "rh_stepper",
                               args.right_amp, args.right_freq, args.right_phase,
                               args.right_offset, args.right_center))
    if not axes:
        print("Nothing to do: pass --left and/or --right")
        return

    rclpy.init()
    node = SinePublisher(axes, rate_hz=args.rate, duration=args.duration, send_vel_dot=args.send_vel_dot)

    # Optional: stop on ENTER when duration is not provided
    stop_with_enter = args.duration is None

    def spin():
        while rclpy.ok() and not node.stop_flag:
            rclpy.spin_once(node, timeout_sec=0.1)

    t = threading.Thread(target=spin, daemon=True)
    t.start()

    if stop_with_enter:
        print("Publishing. Press ENTER to stop…")
        try:
            input()
        except KeyboardInterrupt:
            pass
        node.stop()
    t.join()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
