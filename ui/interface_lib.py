import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist

import miro_msgs
from miro_msgs.msg import platform_config, platform_sensors, platform_state, platform_mics, platform_control, \
    core_state, core_control, core_config, bridge_config, bridge_stream

###############################################################

def fmt(x, f):
    s = ""
    x = bytearray(x)
    for i in range(0, len(x)):
        if not i == 0:
            s = s + ", "
        s = s + f.format(x[i])
    return s

def hex2(x):
    return "{0:#04x}".format(x)

def hex4(x):
    return "{0:#06x}".format(x)

def hex8(x):
    return "{0:#010x}".format(x)

def get_i2c_comms_text(q):
    s = hex8(q.success_r) + " / " + hex8(q.success_w)
    if (s == "0x00c4dca6 / 0x0004c786"):
        s = "(SHLOFF) " + s
    if (s == "0x00c4dce6 / 0x0004c786"):
        s = "(PASS) " + s
    return s

def flt3(x):
    return "{0:.3f}".format(x)

def format_fps(fps, uncompressed):
    text = '{0:.1f}'.format(fps)
    if uncompressed:
        text = text + " (uncompressed)"
    else:
        text = text + " (compressed)"
    return text

def firmware_string_sub(code):
    s = "R"
    if ((code & 0x8000) >> 15): s = "U"
    y = ((code & 0x7E00) >> 9)
    m = ((code & 0x01E0) >> 5)
    d = ((code & 0x001F) >> 0)
    return s+"{0:#02d}".format(y)+"{0:#02d}".format(m)+"{0:#02d}".format(d)

def firmware_string(state):
    a = firmware_string_sub(state.P1_release)
    b = firmware_string_sub(state.P2_release)
    c = firmware_string_sub(state.P2_bootloader_release)
    d = firmware_string_sub(state.P3_release)
    s = a + " / " + b + " / " + c + " / " + d
    if (a == b and a == c and a == d):
        s = "(MATCH) " + s
    return s

def error(msg):
    print(msg)
    sys.exit(0)


def usage():
    print """
Usage:
    miro_ros_client.py robot=<robot_name>

    Without arguments, this help page is displayed. To run the
    client you must specify at least the option "robot".

Options:
    robot=<robot_name>
        specify the name of the miro robot to connect to,
        which forms the ros base topic "/miro/<robot_name>".
        there is no default, this argument must be specified.
    """
    sys.exit(0)


################################################################
class miro_ros_client:
    def callback_platform_sensors(self, object):

        # ignore until active
        if not self.active:
            return

        # store object
        self.platform_sensors = object

    def callback_platform_state(self, object):

        # ignore until active
        if not self.active:
            return

        # store object
        self.platform_state = object

    def callback_platform_mics(self, object):

        # ignore until active
        if not self.active:
            return

        # store object
        self.platform_mics = object

    def callback_core_state(self, object):

        # ignore until active
        if not self.active:
            return

        # store object
        self.core_state = object

    def __init__(self):

        # report
        print("initialising...")
        print(sys.version)

        # default data
        self.platform_sensors = None
        self.platform_state = None
        self.platform_mics = None
        self.core_state = None

        # no arguments gives usage
        if len(sys.argv) == 1:
            usage()

        # options
        self.robot_name = ""
        self.drive_pattern = ""

        # handle args
        for arg in sys.argv[1:]:
            f = arg.find('=')
            if f == -1:
                key = arg
                val = ""
            else:
                key = arg[:f]
                val = arg[f + 1:]
            if key == "robot":
                self.robot_name = val
            elif key == "drive":
                self.drive_pattern = val
            else:
                error("argument not recognised \"" + arg + "\"")

        # check we got at least one
        if len(self.robot_name) == 0:
            error("argument \"robot\" must be specified")

        # set inactive
        self.active = False

        # topic root
        topic_root = "/miro/" + self.robot_name
        print "topic_root", topic_root

        # publish
        self.pub_platform_control = rospy.Publisher(topic_root + "/platform/control", platform_control, queue_size=0)
        self.pub_core_control = rospy.Publisher(topic_root + "/core/control", core_control, queue_size=0)
        self.pub_core_config = rospy.Publisher(topic_root + "/core/config", core_config, queue_size=0)
        self.pub_bridge_config = rospy.Publisher(topic_root + "/bridge/config", bridge_config, queue_size=0)
        self.pub_bridge_stream = rospy.Publisher(topic_root + "/bridge/stream", bridge_stream, queue_size=0)
        self.pub_platform_config = rospy.Publisher(topic_root + "/platform/config", platform_config, queue_size=0)

        # subscribe
        self.sub_sensors = rospy.Subscriber(topic_root + "/platform/sensors", platform_sensors, self.callback_platform_sensors)
        self.sub_state = rospy.Subscriber(topic_root + "/platform/state", platform_state, self.callback_platform_state)
        self.sub_mics = rospy.Subscriber(topic_root + "/platform/mics", platform_mics, self.callback_platform_mics)
        self.sub_core_state = rospy.Subscriber(topic_root + "/core/state", core_state, self.callback_core_state)

        # set active
        self.active = True

    #==================================================
    def update_data(self):

        # sensors
        q = self.platform_sensors
        self.platform_sensors = None
        if hasattr(q, 'battery_state'):
            self.vbat = q.battery_state.voltage
        else:
            self.vbat = q.battery_voltage
        self.vtemp = q.temperature.temperature
        self.eyelid_closure = q.eyelid_closure
        self.sonar = q.sonar_range.range
        self.accel_head = q.accel_head.linear_acceleration
        self.accel_body = q.accel_body.linear_acceleration.x
        self.odomx = q.odometry.twist.twist.linear.x
        self.odomz = q.odometry.twist.twist.angular.z

        self.text_joints = q.joint_state.position
        self.text_joints_effort = q.joint_state.effort
        self.light = q.light
        self.touch_head = q.touch_head
        self.touch_body = q.touch_body
        self.cliff = q.cliff
        self.dip_state = hex2(q.dip_state_phys)

        # state
        q = self.platform_state
        self.platform_state = None
        self.time = q.time_usec * 1e-6
        self.sound_index_P3 = q.number_of_loaded_sounds
        self.P1_R_signals = q.P1_R_signals
        self.P2C_R_signals = q.P2C_R_signals
        self.P2L_R_signals = q.P2L_R_signals
        self.P2U_R_signals = q.P2U_R_signals
        self.text_I2C_comms = get_i2c_comms_text(q)
        self.text_rng_seed = hex8(q.seed)
        self.text_P1_error_code = hex2(q.P1_error_code)
        self.text_firmware = firmware_string(q)
        self.text_mode.set_text(str(q.P1_mode) + " / " + str(q.P2_mode))
        self.text_serial.set_text(str.format('{:04d}', q.serial_number))
        self.text_num_free_stream_buf.set_text(str(q.num_free_stream_buf) + " (" + str(q.msg_id_of_last_stream_buf_recv) + ")")

def hex2(x):
    return "{0:#04x}".format(x)