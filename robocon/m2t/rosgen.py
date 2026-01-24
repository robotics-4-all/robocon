from os import path, mkdir, getcwd, chmod
import shutil

import jinja2


_THIS_DIR = path.abspath(path.dirname(__file__))


# Initialize template engine.
jinja_env = jinja2.Environment(
    loader=jinja2.FileSystemLoader(path.join(_THIS_DIR, "..", "templates")),
    trim_blocks=True,
    lstrip_blocks=True
)


class GeneratorROS:
    bridge_tpl = jinja_env.get_template('ros_bridge.py.j2')
    reqs_tpl = jinja_env.get_template('requirements.txt.j2')
    safe_eval_tpl = jinja_env.get_template('safe_eval.py')
    srcgen_folder = path.join(getcwd(), 'gen')
    
    PY_DEPS = [
        ('commlib-py', '>=', '0.11.2'),
        ('ros-msg-transform', '>=', '0.1.0')
    ]

    @staticmethod
    def generate(model: object, out_dir: str = None):
        if not path.exists(out_dir):
            mkdir(out_dir)
        out_file = path.join(out_dir, f"{model.robot.name}_bridge.py")
        if model.robot.type != 'ROS':
            print('[ERROR] - Did not find any ROS System definition!')
            return
        GeneratorROS.report(model)
        code = GeneratorROS.bridge_tpl.render(model=model)
        with open(out_file, 'w') as f:
            f.write(code)
        # Give execution permissions to the generated file
        chmod(out_file, 509)
        GeneratorROS.gen_requirements(out_dir)
        # Copy safe_eval.py to output directory
        GeneratorROS.copy_safe_eval(out_dir)
        # Copy Dockerfile to output directory
        GeneratorROS.copy_dockerfile(out_dir)
        
    @staticmethod
    def copy_safe_eval(out_dir):
        """Copy safe_eval.py to output directory."""
        src = path.join(path.dirname(__file__), '..', 'templates', 'safe_eval.py')
        dst = path.join(out_dir, 'safe_eval.py')
        shutil.copy(src, dst)

    @staticmethod
    def copy_dockerfile(out_dir):
        """Copy Dockerfile to output directory."""
        src = path.join(path.dirname(__file__), '..', 'templates', 'Dockerfile.ros')
        dst = path.join(out_dir, 'Dockerfile')
        shutil.copy(src, dst)

    @staticmethod
    def gen_requirements(out_dir):
        out_file = path.join(out_dir, "requirements.txt")
        contents = GeneratorROS.reqs_tpl.render(deps=GeneratorROS.PY_DEPS)
        with open(out_file, 'w') as f:
            f.write(contents)

    @staticmethod
    def report(model):
        print(f"[*] - ROS System: {model.robot.name}")
        for bridge in model.bridges:
            if bridge.__class__.__name__ == 'TFBridge':
                ros_uri = 'multiple'
                direction = 'R2B'
                broker_uri = f"{bridge.prefix}/*"
            else:
                if hasattr(bridge, 'topic'):
                    ros_uri = bridge.topic.uri
                elif hasattr(bridge, 'service'):
                    ros_uri = bridge.service.uri
                else:
                    ros_uri = getattr(bridge, 'rosURI', 'N/A')
                direction = bridge.direction
                broker_uri = bridge.brokerURI
            print(f'[*] - Bridge: Type={bridge.__class__.__name__},' + \
                  f' Direction={direction}, ROS_URI={ros_uri},' + \
                  f' Broker_URI={broker_uri},' + \
                  f' Broker=<{model.broker.host}:{model.broker.port}>')
