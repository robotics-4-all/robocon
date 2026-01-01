import sys
from os import path, mkdir, getcwd, chmod


import jinja2

from roboconnect.utils import build_model

_THIS_DIR = path.abspath(path.dirname(__file__))


# Initialize template engine.
jinja_env = jinja2.Environment(
    loader=jinja2.FileSystemLoader(path.join(_THIS_DIR, "..", 'templates')),
    trim_blocks=True,
    lstrip_blocks=True
)


class GeneratorROS2:
    bridge_tpl = jinja_env.get_template('ros2_bridge.j2')
    reqs_tpl = jinja_env.get_template('requirements.txt.j2')
    srcgen_folder = path.join(getcwd(), 'gen')

    PY_DEPS = [
        ('commlib-py', '>=', '0.11.2'),
        ('ros2-msg-transform', '>=', '0.2.2')
    ]

    @staticmethod
    def generate(model: object, out_dir: str = "gen"):
        if not path.exists(out_dir):
            mkdir(out_dir)
        out_file = path.join(out_dir, f"{model.robot.name}_bridges.py")
        if model.robot.type != 'ROS2':
            print('[ERROR] - Did not find any ROS2 System definition!')
            return
        GeneratorROS2.report(model)
        code = GeneratorROS2.bridge_tpl.render(model=model)
        with open(out_file, 'w') as f:
            f.write(code)
        # Give execution permissions to the generated file
        chmod(out_file, 509)
        GeneratorROS2.gen_requirements(out_dir)

    @staticmethod
    def gen_requirements(out_dir):
        out_file = path.join(out_dir, "requirements.txt")
        contents = GeneratorROS2.reqs_tpl.render(deps=GeneratorROS2.PY_DEPS)
        with open(out_file, 'w') as f:
            f.write(contents)

    @staticmethod
    def report(model):
        print(f"[*] - ROS2 System: {model.robot.name}")
        for bridge in model.bridges:
            if hasattr(bridge, 'topic'):
                ros_uri = bridge.topic.uri
            elif hasattr(bridge, 'service'):
                ros_uri = bridge.service.uri
            else:
                ros_uri = getattr(bridge, 'rosURI', 'N/A')
            print(f'[*] - Bridge: Type={bridge.__class__.__name__},' + \
                  f' Direction={bridge.direction}, ROS_URI={ros_uri},' + \
                  f' Broker_URI={bridge.brokerURI},' + \
                  f' Broker=<{model.broker.host}:{model.broker.port}>')
