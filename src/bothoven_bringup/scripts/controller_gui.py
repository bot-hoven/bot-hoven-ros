# Minimal GUI to command left/right solenoids (on/off),
# servos (position, 5 mini sliders), and steppers (position setpoint).
# Topics:
# - Solenoids: /<side>_solenoid_controller/commands (std_msgs/Float64MultiArray, 5 vals)
# - Servos:    /<side>_servo_controller/commands    (std_msgs/Float64MultiArray, 5 vals)
# - Steppers:  /<side>_stepper_controller/reference  (control_msgs/MultiDOFCommand)

import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from control_msgs.msg import MultiDOFCommand

LEFT = "left"
RIGHT = "right"

SOLENOID_JOINTS = {
    LEFT:  ["lh_solenoid_1", "lh_solenoid_2", "lh_solenoid_3", "lh_solenoid_4", "lh_solenoid_5"],
    RIGHT: ["rh_solenoid_1", "rh_solenoid_2", "rh_solenoid_3", "rh_solenoid_4", "rh_solenoid_5"],
}
SOLENOID_TOPIC = {
    LEFT:  "/left_solenoid_controller/commands",
    RIGHT: "/right_solenoid_controller/commands",
}

SERVO_JOINTS = {
    LEFT:  ["lh_servo_1", "lh_servo_2", "lh_servo_3", "lh_servo_4", "lh_servo_5"],
    RIGHT: ["rh_servo_1", "rh_servo_2", "rh_servo_3", "rh_servo_4", "rh_servo_5"],
}
SERVO_TOPIC = {
    LEFT:  "/left_servo_controller/commands",
    RIGHT: "/right_servo_controller/commands",
}

SERVO_LIMITS = (-30.0, 30.0)

STEPPER_DOF = {LEFT: "lh_stepper", RIGHT: "rh_stepper"}
STEPPER_TOPIC = {
    LEFT:  "/left_stepper_controller/reference",
    RIGHT: "/right_stepper_controller/reference",
}
STEPPER_LIMITS = {
    LEFT:  (-0.3096, 0.7064),
    RIGHT: (-0.7064, 0.3096),
}

# how often we publish slider values (Hz)
SLIDER_PUB_RATE_HZ = 20.0


class HandUINode(Node):
    def __init__(self):
        super().__init__("hand_gui")
        self.solenoid_pub = {
            LEFT:  self.create_publisher(Float64MultiArray, SOLENOID_TOPIC[LEFT], 10),
            RIGHT: self.create_publisher(Float64MultiArray, SOLENOID_TOPIC[RIGHT], 10),
        }
        self.servo_pub = {
            LEFT:  self.create_publisher(Float64MultiArray, SERVO_TOPIC[LEFT], 10),
            RIGHT: self.create_publisher(Float64MultiArray, SERVO_TOPIC[RIGHT], 10),
        }
        self.stepper_pub = {
            LEFT:  self.create_publisher(MultiDOFCommand, STEPPER_TOPIC[LEFT], 10),
            RIGHT: self.create_publisher(MultiDOFCommand, STEPPER_TOPIC[RIGHT], 10),
        }

    def publish_solenoids(self, side: str, states_01):
        msg = Float64MultiArray()
        msg.data = [float(x) for x in states_01]
        self.solenoid_pub[side].publish(msg)

    def publish_servos(self, side: str, positions):
        lo, hi = SERVO_LIMITS
        msg = Float64MultiArray()
        msg.data = [max(lo, min(hi, float(x))) for x in positions]
        self.servo_pub[side].publish(msg)

    def publish_stepper(self, side: str, position: float):
        lo, hi = STEPPER_LIMITS[side]
        pos = max(lo, min(hi, float(position)))
        msg = MultiDOFCommand()
        msg.dof_names = [STEPPER_DOF[side]]
        msg.values = [pos]
        self.stepper_pub[side].publish(msg)


# Tk GUI
class HandPane(ttk.Frame):
    def __init__(self, master, side: str, ros: HandUINode):
        super().__init__(master)
        self.side = side
        self.ros = ros
        self.configure(style="Card.TFrame")

        accent = "#1f6feb"

        title = ttk.Label(self, text=f"{side.capitalize()} Hand", style="Title.TLabel")
        title.grid(row=0, column=0, columnspan=9, sticky="w", padx=6, pady=(6, 2))

        # Row 1: Solenoids
        ttk.Label(self, text="Solenoids:", foreground=accent).grid(row=1, column=0, padx=6, pady=6, sticky="w")
        self.sol_vars = []
        for i, _ in enumerate(SOLENOID_JOINTS[side], start=1):
            v = tk.IntVar(value=0)
            btn = ttk.Checkbutton(
                self, text=str(i), variable=v, command=self._on_sol_change,
                style="Toggle.TCheckbutton", width=3
            )
            btn.grid(row=1, column=i, padx=4, pady=6)
            self.sol_vars.append(v)
        off_btn = ttk.Button(self, text="All Off", command=self._all_off, style="Accent.TButton")
        off_btn.grid(row=1, column=7, padx=(12, 6))

        # Row 2: Servos (five miniature sliders)
        ttk.Label(self, text="Servos:", foreground=accent).grid(row=2, column=0, padx=6, pady=(0, 8), sticky="w")

        self.servo_vars = []
        self.servo_reads = []
        lo_servo, hi_servo = SERVO_LIMITS
        # Small sliders: shorter length so five fit comfortably in one row
        MINI_LEN = 110

        for i in range(5):
            v = tk.DoubleVar(value=0.0)
            self.servo_vars.append(v)
            # label above each slider (S1..S5)
            lbl = ttk.Label(self, text=f"S{i+1}")
            lbl.grid(row=2, column=i+1, padx=2, pady=(0, 0), sticky="s")

            s = ttk.Scale(
                self, from_=lo_servo, to=hi_servo, orient="horizontal",
                length=MINI_LEN, variable=v,
                command=self._on_servo_move
            )
            s.grid(row=3, column=i+1, padx=4, pady=(0, 2), sticky="we")

            rd = ttk.Label(self, text="0.000", width=6, anchor="e")
            rd.grid(row=4, column=i+1, padx=4, pady=(0, 6), sticky="e")
            self.servo_reads.append(rd)

        zero_btn = ttk.Button(self, text="Zero All", command=self._servos_zero, style="Accent.TButton")
        zero_btn.grid(row=3, column=7, rowspan=2, padx=(12, 6), pady=(0, 6), sticky="ns")

        self.servo_dirty = False
        # periodic publisher for servos
        servo_period_ms = int(1000.0 / SLIDER_PUB_RATE_HZ)
        self.after(servo_period_ms, self._servos_pump, servo_period_ms)

        # Row 3: Stepper slider (now row 5 to leave space for servo rows 2–4)
        ttk.Label(self, text="Stepper pos:", foreground=accent).grid(row=5, column=0, padx=6, pady=(2, 10), sticky="w")

        lo, hi = STEPPER_LIMITS[side]
        self.slider_val = tk.DoubleVar(value=0.0)
        self.slider_dirty = False

        self.slider = ttk.Scale(
            self, from_=lo, to=hi, orient="horizontal",
            variable=self.slider_val, command=self._on_stepper_slider
        )
        self.slider.grid(row=5, column=1, columnspan=6, sticky="we", padx=6, pady=(2, 10))

        self.readout = ttk.Label(self, text="0.0000", width=10, anchor="e")
        self.readout.grid(row=5, column=7, padx=(6, 10), pady=(2, 10), sticky="e")

        # expand horizontally
        self.columnconfigure(6, weight=1)

        # periodic publisher for the stepper
        stepper_period_ms = int(1000.0 / SLIDER_PUB_RATE_HZ)
        self.after(stepper_period_ms, self._stepper_pump, stepper_period_ms)

    # Solenoids
    def _on_sol_change(self):
        states = [var.get() for var in self.sol_vars]  # 0/1
        self.ros.publish_solenoids(self.side, states)

    def _all_off(self):
        for v in self.sol_vars:
            v.set(0)
        self._on_sol_change()

    # Servos
    def _on_servo_move(self, _evt=None):
        for v, rd in zip(self.servo_vars, self.servo_reads):
            rd.config(text=f"{v.get():+.3f}")
        self.servo_dirty = True

    def _servos_zero(self):
        for v in self.servo_vars:
            v.set(0.0)
        self._on_servo_move()

    def _servos_pump(self, period_ms):
        if self.servo_dirty:
            self.servo_dirty = False
            vals = [v.get() for v in self.servo_vars]
            self.ros.publish_servos(self.side, vals)
        self.after(period_ms, self._servos_pump, period_ms)

    # Stepper
    def _on_stepper_slider(self, _evt=None):
        val = self.slider_val.get()
        self.readout.config(text=f"{val:+.4f}")
        self.slider_dirty = True

    def _stepper_pump(self, period_ms):
        if self.slider_dirty:
            self.slider_dirty = False
            self.ros.publish_stepper(self.side, self.slider_val.get())
        self.after(period_ms, self._stepper_pump, period_ms)


def main():
    rclpy.init()
    node = HandUINode()

    root = tk.Tk()
    root.title("Hand Command GUI")
    root.configure(bg="white")
    style = ttk.Style()
    style.theme_use("clam")
    style.configure(".", background="white")
    style.configure("Card.TFrame", background="white", relief="groove", padding=8)
    style.configure("Title.TLabel", font=("TkDefaultFont", 12, "bold"), background="white")
    style.configure("Accent.TButton", padding=(6, 2))
    style.configure("Toggle.TCheckbutton", background="white")

    outer = ttk.Frame(root, style="Card.TFrame")
    outer.pack(fill="both", expand=True, padx=8, pady=8)

    left = HandPane(outer, LEFT, node)
    right = HandPane(outer, RIGHT, node)
    left.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
    right.grid(row=0, column=1, sticky="nsew", padx=(6, 0))
    outer.columnconfigure(0, weight=1)
    outer.columnconfigure(1, weight=1)

    def on_close():
        try:
            node.destroy_node()
            rclpy.shutdown()
        finally:
            root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
