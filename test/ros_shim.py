"""Small transport substitute for offline callback tests, never used by production.

These tests execute real node callbacks and worker code. They do not emulate DDS,
ROS discovery, serialization, scheduling or hardware, which require a ROS run.
"""
import copy
import sys
import time
from types import ModuleType, SimpleNamespace as NS


class Message:
    def __init__(self, **kwargs):
        self.header = NS(stamp=NS(sec=0, nanosec=0), frame_id='')
        self.valid_for = NS(sec=0, nanosec=0)
        self.twist = NS(linear=NS(x=0.0, y=0.0, z=0.0), angular=NS(x=0.0, y=0.0, z=0.0))
        self.data = None
        self.__dict__.update(kwargs)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))

    def get_subscription_count(self):
        return 1


class BridgeMessage(Message):
    def __init__(self, **kwargs):
        super().__init__(connected=False, robot_ready=False, motion_enabled=False,
                         stop_requested=False, stop_confirmed=False, transport_error='',
                         reason='', bridge_epoch='', rejected_messages=0)
        self.__dict__.update(kwargs)


class Client:
    def __init__(self):
        self.calls = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.calls.append(request)
        return NS(add_done_callback=lambda callback: None)


class Node:
    overrides = {}

    def __init__(self, name):
        self.params = {'use_sim_time': False}
        self.name = name

    def declare_parameter(self, name, value, *args):
        self.params[name] = self.overrides.get(name, value)

    def get_parameter(self, name):
        return NS(value=self.params[name])

    def create_publisher(self, *args):
        return Publisher()

    def create_subscription(self, *args):
        return args

    def create_service(self, *args):
        return args

    def create_timer(self, *args):
        return args

    def create_client(self, *args):
        return Client()

    def get_logger(self):
        return NS(info=lambda message: None, warning=lambda message: None, error=lambda message: None)

    def get_clock(self):
        return NS(now=lambda: NS(to_msg=lambda: NS(sec=int(time.time()), nanosec=0)))

    def destroy_node(self):
        pass


def install():
    definitions = {
        'rclpy': {}, 'rclpy.node': {'Node': Node},
        'rclpy.signals': {'SignalHandlerOptions': NS(NO=0)},
        'rclpy.qos': {'QoSProfile': lambda **kw: NS(**kw),
                      'ReliabilityPolicy': NS(RELIABLE=1, BEST_EFFORT=2),
                      'DurabilityPolicy': NS(VOLATILE=1, TRANSIENT_LOCAL=2)},
        'rcl_interfaces.msg': {'ParameterDescriptor': lambda **kw: NS(**kw)},
        'sensor_msgs.msg': {'NavSatFix': Message, 'Image': Message},
        'std_msgs.msg': {'Bool': Message, 'String': Message},
        'std_srvs.srv': {'Trigger': NS(Request=Message)},
        'go2_interfaces.msg': {'BridgeState': BridgeMessage, 'MotionPermit': Message, 'MotionCommand': Message},
    }
    for name, contents in definitions.items():
        if '.' in name:
            root = name.split('.')[0]
            sys.modules.setdefault(root, ModuleType(root))
        mod = ModuleType(name)
        mod.__dict__.update(contents)
        sys.modules[name] = mod
