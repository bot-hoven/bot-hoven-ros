import argparse, pathlib, sys, time, json, yaml
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray
from control_msgs.msg import MultiDOFCommand

NUM_FINGERS = 5

CONTROLLERS = {
    "lh_stepper":  ("left_stepper_controller",  None),
    **{f"lh_servo_{i}": ("left_servo_controller", i-1) for i in range(1, NUM_FINGERS+1)},
    **{f"lh_solenoid_{i}": ("left_solenoid_controller", i-1) for i in range(1, NUM_FINGERS+1)},
    "rh_stepper":  ("right_stepper_controller", None),
    **{f"rh_servo_{i}": ("right_servo_controller", i-1) for i in range(1, NUM_FINGERS+1)},
    **{f"rh_solenoid_{i}": ("right_solenoid_controller", i-1) for i in range(1, NUM_FINGERS+1)},
}

STEP_TOPIC   = "/{}/reference"
ARRAY_TOPIC  = "/{}/commands"

def load_recipe(path: pathlib.Path):
    txt = path.read_text()
    if path.suffix in (".yml", ".yaml"):
        return yaml.safe_load(txt)
    raise ValueError(f"Unsupported recipe type: {path.suffix}")

def parse_pairs(pairs):
    out = {}
    for pair in pairs:
        try:
            name, val = pair.split("=", 1)
            out[name] = float(val)
        except ValueError:
            raise argparse.ArgumentTypeError(f"Bad ACT=VAL syntax: {pair}")
    return out

def main(argv=None):
    parser = argparse.ArgumentParser(description="Send commands to hand actuators")
    parser.add_argument("pairs", nargs="*", metavar="ACT=POS",
                        help="Actuator‑position pairs, e.g. rh_servo_2=1.1")
    parser.add_argument("-r", "--recipe", type=pathlib.Path,
                        help="YAML file with timed commands")
    args = parser.parse_args(argv)

    if not args.pairs and not args.recipe:
        parser.error("Provide at least one ACT=POS pair or --recipe file")

    rclpy.init()
    node = rclpy.create_node("hand_command_sender")

    array_pubs, step_pubs = {}, {}

    def get_array_pub(controller):
        if controller not in array_pubs:
            array_pubs[controller] = node.create_publisher(
                Float64MultiArray, ARRAY_TOPIC.format(controller), 10)
        return array_pubs[controller]

    def get_step_pub(controller):
        if controller not in step_pubs:
            step_pubs[controller] = node.create_publisher(
                MultiDOFCommand, STEP_TOPIC.format(controller), 10)
        return step_pubs[controller]

    def publish(commands: dict):
        """commands: {actuator_name: position}"""
        grouped = {}
        for act, pos in commands.items():
            if act not in CONTROLLERS:
                node.get_logger().error(f"Unknown actuator {act}")
                continue
            ctl, idx = CONTROLLERS[act]
            grouped.setdefault(ctl, {})[act] = (idx, pos)

        for ctl, acts in grouped.items():
            print(ctl)
            print(acts)
            # MultiDOF case (idx is None)
            if any(idx is None for idx, _ in acts.values()):
                msg = MultiDOFCommand()
                for act, (idx, pos) in acts.items():
                    msg.dof_names.append(act)
                    msg.values.append(float(pos))
                get_step_pub(ctl).publish(msg)
            else:
                data = [0.0] * NUM_FINGERS
                for act, (idx, pos) in acts.items():
                    data[idx] = float(pos)
                print(data)
                get_array_pub(ctl).publish(Float64MultiArray(data=data))

    if args.pairs:
        publish(parse_pairs(args.pairs))

    if args.recipe:
        steps = load_recipe(args.recipe)
        if isinstance(steps, dict):
            steps = [steps[k] for k in sorted(steps)]
        start = steps[0].get("time", 0.0)
        for step in steps:
            wait = step.get("time", 0.0) - start
            start = step.get("time", 0.0)
            if wait > 0:
                time.sleep(wait)
            publish(step["commands"])

    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
