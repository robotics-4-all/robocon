import textx
from textx import GeneratorDesc

from roboconnect.m2t.rosgen import GeneratorROS
from roboconnect.m2t.ros2gen import GeneratorROS2


def _generator_ros_impl(metamodel, model, output_path, overwrite,
                        debug, **custom_args):
    # Some code that perform generation
    GeneratorROS.generate(model._tx_filename)


def _generator_ros2_impl(metamodel, model, output_path, overwrite,
                         debug, **custom_args):
    # Some code that perform generation
    GeneratorROS2.generate(model._tx_filename)


generator_ros = GeneratorDesc(
    language='roboconnect',
    target='ros',
    description='Generate ROS Robot Bridges',
    generator=_generator_ros_impl)


generator_ros2 = GeneratorDesc(
    language='roboconnect',
    target='ros2',
    description='Generate ROS2 Robot Bridges',
    generator=_generator_ros2_impl)

