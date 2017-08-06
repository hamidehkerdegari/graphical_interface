#!/usr/bin/env python
#	@file
#	@section COPYRIGHT
#	Copyright (C) 2017 Consequential Robotics (CqR)
#	
#	@section AUTHOR
#	Consequential Robotics http://consequentialrobotics.com
#	
#	@section LICENSE
#	For a full copy of the license agreement, see LICENSE.MDK in
#	the MDK root directory.
#	
#	Subject to the terms of this Agreement, Consequential Robotics
#	grants to you a limited, non-exclusive, non-transferable license,
#	without right to sub-license, to use MIRO Developer Kit in
#	accordance with this Agreement and any other written agreement
#	with Consequential Robotics. Consequential Robotics does not
#	transfer the title of MIRO Developer Kit to you; the license
#	granted to you is not a sale. This agreement is a binding legal
#	agreement between Consequential Robotics and the purchasers or
#	users of MIRO Developer Kit.
#	
#	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
#	EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
#	OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
#	NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
#	HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
#	WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
#	FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
#	OTHER DEALINGS IN THE SOFTWARE.
#	
#	@section DESCRIPTION
#

################################################################

import threading

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk,GObject,Gdk,GLib,GdkPixbuf

GObject.threads_init()
Gdk.threads_init()

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image,CompressedImage

import miro_msgs
from miro_msgs.msg import platform_config,platform_sensors,platform_state,platform_mics,platform_control,core_state,core_control,core_config,bridge_config,bridge_stream

import math
import cv2
import numpy
import copy
import time
import threading
import sys
from miro_constants import miro
from random import randint

#INTERPTYPE = GdkPixbuf.InterpType.NEAREST
INTERPTYPE = GdkPixbuf.InterpType.BILINEAR

################################################################

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

MIRO_CAM_DISTORTION_MODEL_K1 = -0.75
MIRO_CAM_DISTORTION_MODEL_K2 = 0.25
MIRO_CAM_PIXEL_ASPECT_RATIO = 1.0
MIRO_CAM_HORI_HALF_FOV = miro.__DEG2RAD(45)

# We use a quartic barrel distortion model. Pass in the radius
# of a point in the affine projection to find the distortion
# coefficient.
def camera_model_distort_coeff(rad2):
	rad4 = rad2 * rad2
	return 1.0 \
		+ MIRO_CAM_DISTORTION_MODEL_K1 * rad2 \
		+ MIRO_CAM_DISTORTION_MODEL_K2 * rad4

# Apply distortion (map from affine projection space to image
# space).
def camera_model_u2d(u):
    rad = u[0] * u[0] + u[1] * u[1]
    z = camera_model_distort_coeff(rad)
    result = [0] * 2
    result[0] = u[0] * z;
    result[1] = u[1] * z;
    return result

# Map from d (true) to p (pixel).
#
# NB: This is strictly a transform into an image, and the
# image can be any size, so it's unsurprising that we have to
# indicate the image size as a parameter. For efficiency, we
# assume the image is either of size RAW or DEC.
def camera_model_d2p(d, rows, cols):
    # scale by pixel aspect ratio to correct y coordinate
    p = d
    p[1] = p[1] * MIRO_CAM_PIXEL_ASPECT_RATIO
    # scale by image width (yes, for y coordinate too - that's our standard)
    p[0] = p[0] * cols
    p[1] = p[1] * cols
    # transform by image centre (which is non-zero in P-space)
    p[0] = p[0] + 0.5 * cols - 0.5
    p[1] = p[1] + 0.5 * rows - 0.5
    # ok
    return p

def show_overlay(im):
    # The test pattern is a grid at a fixed range.
    patt_step = 20
    patt_lim = 100
    patt_range = 100
    # extract width and height
    cols = im.shape[1]
    rows = im.shape[0]
    # compute half FOV as an object half width at the given range
    hori_half_width = math.tan(MIRO_CAM_HORI_HALF_FOV) * patt_range
    # pre-divide
    hori_half_width_recip = 0.5 / hori_half_width
    # pre-stretch the pinhole projection so that after the barrel
    # distortion, the image is of the expected (correct) scale
    barrel_correc = 1.0 / camera_model_distort_coeff(0.25)
    # for each test pattern point
    u = [0] * 2
    for i in range(-patt_lim, patt_lim+1, patt_step):
        for j in range(-patt_lim, patt_lim+1, patt_step):
            # lay out pattern in undistorted image space
            u[0] = float(i) * hori_half_width_recip
            u[1] = float(j) * hori_half_width_recip
            # apply barrel distortion
            d = camera_model_u2d(u)
            # apply barrel correction
            d[0] = d[0] * barrel_correc
            d[1] = d[1] * barrel_correc
            # apply image acquisition
            p = camera_model_d2p(d, rows, cols)
            # round (deliberately round down because we're going to
            # make a pattern at each pixel that is 2x2)
            xi_ = int(p[0])
            yi_ = int(p[1])
            # loop
            for k in [0, 1]:
                for l in [0, 1]:
                    xi = xi_ + k
                    yi = yi_ + l
                    # write into image
                    if xi >= 0 and xi < cols and yi >= 0 and yi < rows:
                        q = xi * 3 + yi * cols * 3
                        im.data[q] = numpy.uint8(255);
                        im.data[q+1] = numpy.uint8(255);
                        im.data[q+2] = numpy.uint8(255);

def pp2string(pp):
    return flt3(pp.height) + "/" + flt3(pp.size) + " (@" + flt3(pp.azim) + "," + flt3(pp.elev) + ")"

def get_temp_filename(num, key):
    return "/tmp/spatial/" + "{:04d}".format(num) + key

def usage():
    print """
Usage:
    miro_ros_client_gui.py robot=<robot_name> [options]

    Without arguments, this help page is displayed. To run the
    client you must specify at least the option "robot".

Options:
    robot=<robot_name>
        specify the name of the miro robot to connect to,
        which forms the ros base topic "/miro/<robot_name>".
        there is no default, this argument must be specified.

    rob/sim
        Treat the server robot as a physical/simulated robot.
        If the robot_name starts with the substring "rob",
        this is inferred automatically. At time of writing,
        this setting has no impact on behaviour.

    uncompressed
        Connect to uncompressed video streams. If this argument
        is not specified, compressed video streams are used.
    """
    sys.exit(0)

################################################################

class fifo:

    def __init__(self, uncompressed):
        self.N = 1 # raise if we get "overflow in fifo", or to improve fps measurement
        self.buf = [None] * self.N
        self.r = 0
        self.w = 0
        self.tN = 30
        self.ti = 0
        self.td = 10
        self.tt = [None] * self.tN
        self.hz = None
        self.uncompressed = uncompressed
        self.lock = threading.Lock()

    def push(self, frm, do_overlay = False):
        self.lock.acquire()
        try:
            t_frm = time.time()
            if self.buf[self.w] is None:
                if self.uncompressed:
                    pb = GdkPixbuf.Pixbuf.new_from_data(frm.data,
                        GdkPixbuf.Colorspace.RGB, False, 8,
                        frm.width, frm.height, frm.step)
                else:
                    im = cv2.imdecode(numpy.fromstring(frm.data, numpy.uint8),
                                cv2.IMREAD_COLOR)
                    w = im.shape[1]
                    h = im.shape[0]
                    N = h * w
                    for i in range(0, N):
                        tmp = im.data[i*3+0]
                        im.data[i*3+0] = im.data[i*3+2]
                        im.data[i*3+2] = tmp
                    if do_overlay:
                        show_overlay(im)
                    pb = GdkPixbuf.Pixbuf.new_from_data(im.data,
                                GdkPixbuf.Colorspace.RGB,
                                False, 8,
                                w, h, w*3)
                self.buf[self.w] = pb
            else:
                print("**** frame dropped ****")
                pass
            self.tt[self.ti] = t_frm
            ti_bak = self.ti - self.td
            if ti_bak < 0:
                ti_bak = ti_bak + self.tN
            if not self.tt[ti_bak] is None:
                dt = t_frm - self.tt[ti_bak]
                self.hz = self.td / dt
                if self.hz > (self.td + 3) and self.td < (self.tN - 1):
                    self.td = self.td + 1
                    #print "> ", self.td
                if self.hz < (self.td - 3) and self.td > 5:
                    self.td = self.td - 1
                    #print "< ", self.td
            self.ti = (self.ti + 1) % self.tN
        finally:
            self.lock.release()

    def pop(self):
        obj = self.buf[self.r]
        if not obj is None:
            self.buf[self.r] = None
            self.r = (self.r + 1) % self.N
        return obj

    def latest(self):
        ret = None
        while True:
            obj = self.pop()
            if obj is None:
                break
            ret = obj
        return ret

    def freq(self):
        hz = self.hz
        if not hz is None:
            self.hz = None
        return hz

################################################################

class miro_ros_client_gui:

    def on_window_main_destroy(self, object, data=None):
        print("quit with cancel")
        Gtk.main_quit()

    def on_gtk_quit_activate(self, menuitem, data=None):
        print("quit from menu")
        Gtk.main_quit()

    def button_affect(self, object, data=None):
        s = object.get_label().split('\n')
        if s[0] == 'unhappy':
            valence = 0
        if s[0] == 'neutral':
            valence = 0.5
        if s[0] == 'happy':
            valence = 1
        if s[1] == 'relaxed':
            arousal = 0
        if s[1] == 'neutral':
            arousal = 0.5
        if s[1] == 'aroused':
            arousal = 1
        self.mood = [valence, arousal]

    def button_kc_reset(self, object, data=None):
        self.scale_lift.set_value(0)
        self.scale_yaw.set_value(50)
        self.scale_pitch.set_value(self.ini_pitch)

    def button_cosmetic_reset(self, object, data=None):
        self.scale_eyelids.set_value(0)
        self.scale_tail.set_value(50)
        self.scale_ears_0.set_value(0)
        self.scale_ears_1.set_value(0)

    def button_cosmetic_blink(self, object, data=None):
        self.blink = 5 + randint(0,20)
        if randint(0,1) == 0 and self.blink >= 15:
            self.blink = -self.blink
        #print("blink: " + str(self.blink))

    def button_sound_P1(self, object, data=None):
        self.sound_index_P1 = randint(1, 30)

    def button_sound_P2(self, object, data=None):
        self.sound_index_P2 = randint(1, 23)

    def button_send_bridge_config(self, object, data=None):
        self.no_publish_mics = self.chk_no_publish_mics.get_active()
        self.no_publish_cams = self.chk_no_publish_cams.get_active()
        self.send_bridge_config = True

    def button_send_bridge_stream(self, object, data=None):
        self.sound_index_P3 = self.spin_sound_index_P3.get_value()
        # if spinner is set to zero, we'll actually send -1, because this
        # will cancel the sound playing (zero means "no change")
        if self.sound_index_P3 == 0:
            self.sound_index_P3 = -1
        self.send_bridge_stream = True

    def button_send_platform_config(self, object, data=None):
        if self.combo_frame_size.get_active_text() == "NO CHANGE":
            self.frame_size = 0
        else:
            self.frame_size = int(self.combo_frame_size.get_active_text())
        self.frame_rate = int(self.combo_frame_rate.get_active_id())
        self.send_platform_config = True
        # put frame size to "NO CHANGE" so we won't do it twice
        self.combo_frame_size.set_active(0)

    def button_platform_reset(self, object, data=None):
        self.platform_reset = 1
        self.send_platform_config = True

    def button_spatial_record_start(self, object, data=None):
        self.spatial_record = 1

    def button_spatial_record_stop(self, object, data=None):
        self.spatial_record = 0

    def callback_platform_sensors(self, object):
        self.platform_sensors = object

    def callback_platform_state(self, object):
        self.platform_state = object

    def callback_platform_mics(self, object):
        self.platform_mics = object

    def callback_core_state(self, object):
        self.core_state = object

    def redo_config(self, object):
        self.do_config = 1

    def callback_caml(self, frm):
        if not self.chk_show_mics.get_active():
            self.caml_fifo.push(frm, self.chk_show_cam_overlay.get_active())

    def callback_camr(self, frm):
        if not self.chk_show_mics.get_active():
            self.camr_fifo.push(frm, self.chk_show_cam_overlay.get_active())

    def callback_pril(self, frm):
        self.pril_fifo.push(frm)

    def callback_prir(self, frm):
        self.prir_fifo.push(frm)

    def callback_priw(self, frm):
        self.priw_fifo.push(frm)

    def callback_rgbl(self, frm):
        self.rgbl_fifo.push(frm)

    def callback_rgbr(self, frm):
        self.rgbr_fifo.push(frm)

    def format_rtc(self, q):
        s = '{0:#02d}'.format(q.rtc_hrs)
        s = s + ":"
        s = s + '{0:#02d}'.format(q.rtc_min)
        s = s + ":"
        s = s + '{0:#02d}'.format(q.rtc_sec)
        s = s + " ("
        if (q.rtc_skew > 0):
            s = s + "+"
        s = s + '{0:d}'.format(q.rtc_skew)
        s = s + ")"
        return s

    def update_ui(self):

        # sensors
        if not self.platform_sensors is None:
            q = self.platform_sensors
            self.platform_sensors = None
            if hasattr(q, 'battery_state'):
                vbat = q.battery_state.voltage
            else:
                vbat = q.battery_voltage
            text = '{0:.2f}'.format(vbat) + 'V, ' + '{0:.2f}'.format(q.temperature.temperature) + 'C'
            self.text_vbat_temp.set_text(text)
            self.text_eyelid_closure.set_text('{0:.2f}'.format(q.eyelid_closure))
            self.text_sonar.set_text('{0:.2f}'.format(q.sonar_range.range))
            acc = q.accel_head.linear_acceleration
            self.text_accel_head.set_text('{0:.2f}'.format(acc.x)+", "+'{0:.2f}'.format(acc.y)+", "+'{0:.2f}'.format(acc.z))
            acc = q.accel_body.linear_acceleration
            self.text_accel_body.set_text('{0:.2f}'.format(acc.x)+", "+'{0:.2f}'.format(acc.y)+", "+'{0:.2f}'.format(acc.z))
            twist = q.odometry.twist.twist
            self.text_odom.set_text('{0:.2f}'.format(twist.linear.x)+", "+'{0:.2f}'.format(twist.angular.z))
            self.text_joints.set_text(
                '{0:.1f}'.format(q.joint_state.position[0]*57.3)+", "+ \
                '{0:.1f}'.format(q.joint_state.position[1]*57.3)+", "+ \
                '{0:.1f}'.format(q.joint_state.position[2]*57.3)+", "+ \
                '{0:.1f}'.format(q.joint_state.position[3]*57.3))
            self.text_joints_effort.set_text(
                '{0:.1f}'.format(q.joint_state.effort[0])+", "+ \
                '{0:.1f}'.format(q.joint_state.effort[1])+", "+ \
                '{0:.1f}'.format(q.joint_state.effort[2])+", "+ \
                '{0:.1f}'.format(q.joint_state.effort[3]))
            self.text_light.set_text(fmt(q.light, '{0:.0f}'))
            self.text_touch.set_text(fmt(q.touch_head, '{0:.0f}') + " / " + fmt(q.touch_body, '{0:.0f}'))
            self.text_cliff.set_text(fmt(q.cliff, '{0:.0f}'))
            self.text_dip_state.set_text(hex2(q.dip_state_phys))

        # state
        if not self.platform_state is None:
            q = self.platform_state
            self.platform_state = None
            t = '{0:.2f}'.format(q.time_usec * 1e-6)
            t = t + " / "
            t = t + self.format_rtc(q)
            self.text_time.set_text(t)
            self.adj_sound_index_P3.set_upper(q.number_of_loaded_sounds)
            self.text_P1_R_signals.set_text(hex8(q.P1_R_signals))
            self.text_P2C_R_signals.set_text(hex8(q.P2C_R_signals))
            self.text_P2L_R_signals.set_text(hex8(q.P2L_R_signals))
            self.text_P2U_R_signals.set_text(hex8(q.P2U_R_signals))
            self.text_I2C_comms.set_text(get_i2c_comms_text(q))
            self.text_rng_seed.set_text(hex8(q.seed))
            self.text_P1_error_code.set_text(hex2(q.P1_error_code))
            self.text_firmware.set_text(firmware_string(q))
            self.text_mode.set_text(str(q.P1_mode) + " / " + str(q.P2_mode))
            self.text_serial.set_text(str.format('{:04d}', q.serial_number))
            self.text_num_free_stream_buf.set_text(str(q.num_free_stream_buf) + " (" + str(q.msg_id_of_last_stream_buf_recv) + ")")

        # core
        if not self.core_state is None:
            q = self.core_state
            self.core_state = None
            self.text_emotion.set_text(flt3(q.emotion.valence) + ", " + flt3(q.emotion.arousal))
            self.text_mood.set_text(flt3(q.mood.valence) + ", " + flt3(q.mood.arousal))
            self.text_sleep.set_text(flt3(q.sleep.wakefulness) + ", " + flt3(q.sleep.pressure))
            self.text_pp_0.set_text(pp2string(q.priority_peak[0]))
            self.text_pp_1.set_text(pp2string(q.priority_peak[1]))
            self.text_pp_2.set_text(pp2string(q.priority_peak[2]))

        # lock
        self.lock.acquire()

        # mics
        if self.chk_show_mics.get_active():
            if not self.platform_mics is None:
                q = self.platform_mics
                self.platform_mics = None
                data = q.data
                w_mics = self.w_mics
                h_mics = self.h_mics
                p1 = h_mics / 4
                p2 = p1 * 3
                o = p1 - 1
                s = w_mics * 3
                for j in range(0, w_mics*h_mics*3):
                    self.ima[j] = 0
                    #self.imb[j] = 0
                for j in range(0, w_mics):
                    i = j * 2000 / w_mics
                    a = data[i*2+0]
                    b = data[i*2+1]
                    a = a * o / 512
                    b = b * o / 512
                    x = i * w_mics / 2000
                    self.ima[x*3+(p1-a)*s] = 255
                    self.ima[x*3+(p2-b)*s+1] = 255
                self.pba = GdkPixbuf.Pixbuf.new_from_data(self.ima,
                    GdkPixbuf.Colorspace.RGB, False, 8, w_mics, h_mics,
                    w_mics*3, None, None)
                self.image_caml.set_from_pixbuf(self.pba)
                #self.pbb = GdkPixbuf.Pixbuf.new_from_data(self.imb,
                #    GdkPixbuf.Colorspace.RGB, False, 8, w_mics, h_mics,
                #    w_mics*3, None, None)
                #self.image_camr.set_from_pixbuf(self.pba)

        # cameras
        pb = self.caml_fifo.latest()
        if not pb is None:
            self.image_caml.set_from_pixbuf(pb)
        pb = self.camr_fifo.latest()
        if not pb is None:
            self.image_camr.set_from_pixbuf(pb)
        fps = self.caml_fifo.freq()
        if not fps is None:
            self.text_caml_fps.set_text(format_fps(fps, self.opt.uncompressed))
        fps = self.camr_fifo.freq()
        if not fps is None:
            self.text_camr_fps.set_text(format_fps(fps, self.opt.uncompressed))

        # priority frames
        pb = self.pril_fifo.latest()
        if not pb is None:
            pb = pb.scale_simple(128, 96, INTERPTYPE)
            self.image_pril.set_from_pixbuf(pb)
            if self.spatial_record > 0:
                filename = get_temp_filename(self.spatial_record, "_l.png")
                pb.savev(filename, "png", [], [])
                self.spatial_record = self.spatial_record + 1
        pb = self.prir_fifo.latest()
        if not pb is None:
            pb = pb.scale_simple(128, 96, INTERPTYPE)
            self.image_prir.set_from_pixbuf(pb)
#            if self.spatial_record > 0:
#                filename = get_temp_filename(self.spatial_record, "_r.png")
#                pb.savev(filename, "png", [], [])
#                self.spatial_record = self.spatial_record + 1
        pb = self.priw_fifo.latest()
        if not pb is None:
            pb = pb.scale_simple(320, 16, INTERPTYPE)
            self.image_priw.set_from_pixbuf(pb)
#            if self.spatial_record > 0:
#                filename = get_temp_filename(self.spatial_record, "_w.png")
#                pb.savev(filename, "png", [], [])
#                self.spatial_record = self.spatial_record + 1

        # rgb frames
        pb = self.rgbl_fifo.latest()
        if not pb is None:
            pb = pb.scale_simple(128, 96, INTERPTYPE)
            self.image_pril.set_from_pixbuf(pb)
        pb = self.rgbr_fifo.latest()
        if not pb is None:
            pb = pb.scale_simple(128, 96, INTERPTYPE)
            self.image_prir.set_from_pixbuf(pb)

        # release
        self.lock.release()

        # config
        if self.do_config:
            self.do_config = self.do_config - 1
            if self.do_config == 0:
                c = core_config()
                if self.chk_branch_enable.get_active():
                     c.P2B_W_signals = c.P2B_W_signals | miro.MIRO_P2B_W_BRANCH_ENABLE
                if self.chk_affect_enable.get_active():
                     c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_ENABLE
                if self.chk_affect_adjust_rtc.get_active():
                     c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_ADJUST_RTC
                if self.chk_affect_valence_dynamics.get_active():
                     c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_VALENCE_DYNAMICS
                if self.chk_affect_arousal_dynamics.get_active():
                     c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_AROUSAL_DYNAMICS
                if self.chk_affect_enable_sleep.get_active():
                     c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_ENABLE_SLEEP
                if self.chk_affect_from_clock.get_active():
                     c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_FROM_CLOCK
                if self.chk_affect_from_wakefulness.get_active():
                     c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_FROM_WAKEFULNESS
                if self.chk_affect_from_touch.get_active():
                     c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_FROM_TOUCH
                if self.chk_affect_from_light.get_active():
                     c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_FROM_LIGHT
                if self.chk_affect_from_sound.get_active():
                     c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_FROM_SOUND
                if self.chk_affect_from_accel.get_active():
                     c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_FROM_ACCEL
                if self.chk_affect_from_sleep_blocked.get_active():
                     c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_FROM_SLEEP_BLOCKED
                if self.chk_affect_randomize_valence.get_active():
                     c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_RANDOMIZE_VALENCE
                if self.chk_affect_fast_sleep_dynamics.get_active():
                     c.P2U_W_affect_signals = c.P2U_W_affect_signals | miro.MIRO_P2U_W_AFFECT_FAST_SLEEP_DYNAMICS
                if self.chk_express_enable.get_active():
                     c.P2U_W_express_signals = c.P2U_W_express_signals | miro.MIRO_P2U_W_EXPRESS_ENABLE
                if self.chk_express_through_light.get_active():
                     c.P2U_W_express_signals = c.P2U_W_express_signals | miro.MIRO_P2U_W_EXPRESS_THROUGH_LIGHT
                if self.chk_express_through_tail.get_active():
                     c.P2U_W_express_signals = c.P2U_W_express_signals | miro.MIRO_P2U_W_EXPRESS_THROUGH_TAIL
                if self.chk_express_through_eyelids.get_active():
                     c.P2U_W_express_signals = c.P2U_W_express_signals | miro.MIRO_P2U_W_EXPRESS_THROUGH_EYELIDS
                if self.chk_express_through_ears.get_active():
                     c.P2U_W_express_signals = c.P2U_W_express_signals | miro.MIRO_P2U_W_EXPRESS_THROUGH_EARS
                if self.chk_express_through_vocal.get_active():
                     c.P2U_W_express_signals = c.P2U_W_express_signals | miro.MIRO_P2U_W_EXPRESS_THROUGH_VOCAL
                if self.chk_express_through_body.get_active():
                     c.P2U_W_express_signals = c.P2U_W_express_signals | miro.MIRO_P2U_W_EXPRESS_THROUGH_BODY
                if self.chk_express_through_ping.get_active():
                     c.P2U_W_express_signals = c.P2U_W_express_signals | miro.MIRO_P2U_W_EXPRESS_THROUGH_PING
                if self.chk_express_no_pirate_noises.get_active():
                     c.P2U_W_express_signals = c.P2U_W_express_signals | miro.MIRO_P2U_W_EXPRESS_NO_PIRATE_NOISES
                if self.chk_express_do_pirate_noises.get_active():
                     c.P2U_W_express_signals = c.P2U_W_express_signals | miro.MIRO_P2U_W_EXPRESS_DO_PIRATE_NOISES
                if self.chk_action_enable.get_active():
                     c.P2U_W_action_signals = c.P2U_W_action_signals | miro.MIRO_P2U_W_ACTION_ENABLE
                if self.chk_action_debug.get_active():
                     c.P2U_W_action_signals = c.P2U_W_action_signals | miro.MIRO_P2U_W_ACTION_DEBUG
                if self.chk_action_force_mull.get_active():
                     c.P2U_W_action_signals = c.P2U_W_action_signals | miro.MIRO_P2U_W_ACTION_FORCE_MULL
                if self.chk_action_randomize_orient.get_active():
                     c.P2U_W_action_signals = c.P2U_W_action_signals | miro.MIRO_P2U_W_ACTION_RANDOMIZE_ORIENT
                if self.chk_action_disable_halt.get_active():
                     c.P2U_W_action_signals = c.P2U_W_action_signals | miro.MIRO_P2U_W_ACTION_DISABLE_HALT
                if self.chk_action_modulate_by_sonar.get_active():
                     c.P2U_W_action_signals = c.P2U_W_action_signals | miro.MIRO_P2U_W_ACTION_MODULATE_BY_SONAR
                if self.chk_body_enable.get_active():
                     c.P2U_W_body_signals = c.P2U_W_body_signals | miro.MIRO_P2U_W_BODY_ENABLE
                if self.chk_body_reset_kc_integrators.get_active():
                     c.P2U_W_body_signals = c.P2U_W_body_signals | miro.MIRO_P2U_W_BODY_RESET_KC_INTEGRATORS
                if self.chk_body_no_push.get_active():
                     c.P2U_W_body_signals = c.P2U_W_body_signals | miro.MIRO_P2U_W_BODY_NO_PUSH
                if self.chk_body_no_push_motion.get_active():
                     c.P2U_W_body_signals = c.P2U_W_body_signals | miro.MIRO_P2U_W_BODY_NO_PUSH_MOTION
                if self.chk_body_no_push_translation.get_active():
                     c.P2U_W_body_signals = c.P2U_W_body_signals | miro.MIRO_P2U_W_BODY_NO_PUSH_TRANSLATION
                if self.chk_body_no_push_into_sonar.get_active():
                     c.P2U_W_body_signals = c.P2U_W_body_signals | miro.MIRO_P2U_W_BODY_NO_PUSH_INTO_SONAR
                if self.chk_enable_pos_control.get_active():
                     c.P2L_W_signals = c.P2L_W_signals | miro.MIRO_P2L_W_ENABLE_POS_CONTROL
                if self.chk_enable_cliff_reflex.get_active():
                     c.P2L_W_signals = c.P2L_W_signals | miro.MIRO_P2L_W_ENABLE_CLIFF_REFLEX
                if self.chk_spatial_enable.get_active():
                     c.P2S_W_signals = c.P2S_W_signals | miro.MIRO_P2S_W_SPATIAL_ENABLE
                if self.chk_spatial_ignore_audio.get_active():
                     c.P2S_W_signals = c.P2S_W_signals | miro.MIRO_P2S_W_SPATIAL_IGNORE_AUDIO
                if self.chk_spatial_ignore_video.get_active():
                     c.P2S_W_signals = c.P2S_W_signals | miro.MIRO_P2S_W_SPATIAL_IGNORE_VIDEO
                if self.chk_spatial_send_priority.get_active():
                     c.P2S_W_signals = c.P2S_W_signals | miro.MIRO_P2S_W_SPATIAL_SEND_PRIORITY
                if self.chk_spatial_send_other.get_active():
                     c.P2S_W_signals = c.P2S_W_signals | miro.MIRO_P2S_W_SPATIAL_SEND_OTHER
                if self.chk_spatial_no_reaff_compromise.get_active():
                     c.P2S_W_signals = c.P2S_W_signals | miro.MIRO_P2S_W_SPATIAL_NO_REAFF_COMPROMISE
                if self.chk_spatial_no_suppress.get_active():
                     c.P2S_W_signals = c.P2S_W_signals | miro.MIRO_P2S_W_SPATIAL_NO_SUPPRESS
                if self.chk_spatial_show_compromise.get_active():
                     c.P2S_W_signals = c.P2S_W_signals | miro.MIRO_P2S_W_SPATIAL_SHOW_COMPROMISE
                if self.chk_spatial_show_test_pattern.get_active():
                     c.P2S_W_signals = c.P2S_W_signals | miro.MIRO_P2S_W_SPATIAL_SHOW_TEST_PATTERN
                if self.chk_test_alarm.get_active():
                     c.P1_W_signals = c.P1_W_signals | miro.MIRO_P1_W_TEST_ALARM
                     c.P1_W_signals = c.P1_W_signals | miro.MIRO_P1_W_NO_I2C_BUSY_ALARM
                     #c.P2L_W_signals = c.P2L_W_signals | miro.MIRO_P2L_W_DESTALL_EARS_NOW;
                c.msg_flags = c.FLAG_UPDATE_SIGNALS;
                self.pub_core_config.publish(c)
                print("config sent")

        # bridge/config
        if self.send_bridge_config:
            self.send_bridge_config = False
            q = bridge_config()
            q.flags = miro.MIRO_BRIDGE_UPDATE
            if self.no_publish_mics:
                q.flags = q.flags | miro.MIRO_BRIDGE_NO_PUBLISH_MICS
            if self.no_publish_cams:
                q.flags = q.flags | miro.MIRO_BRIDGE_NO_PUBLISH_CAMS
            self.pub_bridge_config.publish(q)
            print("bridge/config sent")

        # bridge/stream
        if self.send_bridge_stream:
            self.send_bridge_stream = False
            q = bridge_stream()
            q.sound_index_P3 = self.sound_index_P3
            self.pub_bridge_stream.publish(q)
            print("bridge/stream sent")

        # platform/config
        if self.send_platform_config:
            self.send_platform_config = False
            q = platform_config()
            q.reset = self.platform_reset
            self.platform_reset = 0
            q.frame_size = self.frame_size
            max_frame_rate = 0
            if self.frame_size == 128:
                max_frame_rate = 25
            if self.frame_size == 192:
                max_frame_rate = 25
            if self.frame_size == 320:
                max_frame_rate = 8
            if self.frame_size == 640:
                max_frame_rate = 2
            if self.frame_rate == 1:
                q.frame_rate = max_frame_rate * 0.25
            if self.frame_rate == 2:
                q.frame_rate = max_frame_rate * 0.5
            if self.frame_rate == 3:
                q.frame_rate = max_frame_rate * 0.75
            if self.frame_rate == 4:
                q.frame_rate = max_frame_rate
            self.pub_platform_config.publish(q)
            print("platform/config sent")

        # publish
        q = platform_control()
        q.msg_flags = platform_control.FLAG_SYNC_PLATFORM | platform_control.FLAG_SYNC_CORE

        # handle vel buttons
        ang_speed = 3.14
        lin_speed = 200
        if self.button_vel_l.get_state() is Gtk.StateType.ACTIVE:
            q.body_vel.angular.z = +ang_speed
        if self.button_vel_r.get_state() is Gtk.StateType.ACTIVE:
            q.body_vel.angular.z = -ang_speed
        if self.button_vel_f.get_state() is Gtk.StateType.ACTIVE:
            q.body_vel.linear.x = +lin_speed
        if self.button_vel_b.get_state() is Gtk.StateType.ACTIVE:
            q.body_vel.linear.x = -lin_speed

        # handle kc
        if self.chk_drive_kc.get_active():
            x = self.scale_lift.get_value() * 0.01
            q.body_config[1] = x * (miro.MIRO_LIFT_MAX_RAD - miro.MIRO_LIFT_MIN_RAD) + miro.MIRO_LIFT_MIN_RAD
            q.body_config_speed[1] = miro.MIRO_P2U_W_LEAN_SPEED_INF
            x = (100.0 - self.scale_yaw.get_value()) * 0.01
            q.body_config[2] = x * (miro.MIRO_YAW_MAX_RAD - miro.MIRO_YAW_MIN_RAD) + miro.MIRO_YAW_MIN_RAD
            q.body_config_speed[2] = miro.MIRO_P2U_W_LEAN_SPEED_INF
            x = self.scale_pitch.get_value() * 0.01
            q.body_config[3] = x * (miro.MIRO_PITCH_MAX_RAD - miro.MIRO_PITCH_MIN_RAD) + miro.MIRO_PITCH_MIN_RAD
            q.body_config_speed[3] = miro.MIRO_P2U_W_LEAN_SPEED_INF

        # handle cosmetic
        if self.chk_drive_cosmetic.get_active():
            x = self.scale_eyelids.get_value() * 0.01
            q.eyelid_closure = x
            x = self.scale_tail.get_value() * 0.02 - 1.0
            q.tail = x
            x = self.scale_ears_0.get_value() * 0.01
            q.ear_rotate[0] = x
            x = self.scale_ears_1.get_value() * 0.01
            q.ear_rotate[1] = x
            r = int(self.pulse*128-64)
            r = abs(r)
            b = 64 - r
            q.lights_rgb = [r, 0, b]
            q.lights_phase = self.pulse * 255
            q.lights_dphase = 64
            q.lights_amp = 128
            q.lights_max_drive = 128
            q.blink_time = self.blink
            self.blink = 0
            q.sound_index_P1 = self.sound_index_P1
            self.sound_index_P1 = 0
            q.sound_index_P2 = self.sound_index_P2
            self.sound_index_P2 = 0
            # advance pulse
            self.pulse = self.pulse + 0.05
            if self.pulse >= 1:
                self.pulse = 0

        # publish
        self.pub_platform_control.publish(q)

        # publish
        q = core_control()
        q.msg_flags = core_control.FLAG_SYNC_PLATFORM | core_control.FLAG_SYNC_CORE

        # handle push buttons
        push_speed = 100
        q.body_push.append(miro_msgs.msg.push())
        b = miro_msgs.msg.push()
        b.link = miro.MIRO_LINK_HEAD
        b.flags = miro.MIRO_BODY_PUSH_VELOCITY
        b.pushpos.x = 100
        b.pushvec.x = 0
        b.pushvec.y = 0
        b.pushvec.z = 0
        if self.button_push_l.get_state() is Gtk.StateType.ACTIVE:
            b.pushvec.y = push_speed
        if self.button_push_r.get_state() is Gtk.StateType.ACTIVE:
            b.pushvec.y = -push_speed
        if self.button_push_u.get_state() is Gtk.StateType.ACTIVE:
            b.pushvec.z = push_speed
        if self.button_push_d.get_state() is Gtk.StateType.ACTIVE:
            b.pushvec.z = -push_speed
        if self.button_push_f.get_state() is Gtk.StateType.ACTIVE:
            b.pushvec.x = push_speed
        if self.button_push_b.get_state() is Gtk.StateType.ACTIVE:
            b.pushvec.x = -push_speed
        q.body_push.append(b)

        # handle affect
        if not self.mood is None:
            print("mood to " + str(self.mood))
            q.mood_drive_target.valence = self.mood[0]
            q.mood_drive_target.arousal = self.mood[1]
            q.mood_drive_gamma = 1
            q.emotion_drive_target.valence = self.mood[0]
            q.emotion_drive_target.arousal = self.mood[1]
            q.emotion_drive_gamma = 1
            self.mood = None

        # publish
        self.pub_core_control.publish(q)

        return True

    def chk_config(self, label, demo_mode_setting = False):
        obj = self.builder.get_object(label)
        obj.connect("clicked", self.redo_config)
        if demo_mode_setting:
            self.demo_mode_on.append(obj)
        else:
            self.demo_mode_off.append(obj)
        return obj

    def button_config_demo(self, object, data=None):
        print("config: demo")
        for obj in self.demo_mode_on:
            obj.set_active(True)
        for obj in self.demo_mode_off:
            obj.set_active(False)

    def button_config_reset(self, object, data=None):
        print("config: reset")
        for obj in self.demo_mode_on:
            obj.set_active(False)
        for obj in self.demo_mode_off:
            obj.set_active(False)

    def __init__(self):

        # report
        print("initialising...")
        print(sys.version)

        # initialise
        self.builder = Gtk.Builder()
        self.builder.add_from_file("miro_ros_client_gui.glade")

        # get references
        self.window_main = self.builder.get_object("window_main")

        # cameras
        self.image_caml = self.builder.get_object("image_caml")
        self.image_camr = self.builder.get_object("image_camr")
        self.text_caml_fps = self.builder.get_object("text_caml_fps")
        self.text_camr_fps = self.builder.get_object("text_camr_fps")
        self.chk_show_mics = self.builder.get_object("chk_show_mics")
        self.chk_show_cam_overlay = self.builder.get_object("chk_show_cam_overlay")

        # sensors
        self.text_time = self.builder.get_object("text_time")
        self.text_vbat_temp = self.builder.get_object("text_vbat_temp")
        self.text_eyelid_closure = self.builder.get_object("text_eyelid_closure")
        self.text_accel_head = self.builder.get_object("text_accel_head")
        self.text_accel_body = self.builder.get_object("text_accel_body")
        self.text_odom = self.builder.get_object("text_odom")
        self.text_joints = self.builder.get_object("text_joints")
        self.text_joints_effort = self.builder.get_object("text_joints_effort")
        self.text_sonar = self.builder.get_object("text_sonar")
        self.text_light = self.builder.get_object("text_light")
        self.text_touch = self.builder.get_object("text_touch")
        self.text_cliff = self.builder.get_object("text_cliff")

        # state
        self.text_P1_R_signals = self.builder.get_object("text_P1_R_signals")
        self.text_I2C_comms = self.builder.get_object("text_I2C_comms")
        self.text_dip_state = self.builder.get_object("text_dip_state")
        self.text_firmware = self.builder.get_object("text_firmware")
        self.text_mode = self.builder.get_object("text_mode")
        self.text_P1_error_code = self.builder.get_object("text_P1_error_code")
        self.text_P2C_R_signals = self.builder.get_object("text_P2C_R_signals")
        self.text_P2L_R_signals = self.builder.get_object("text_P2L_R_signals")
        self.text_P2U_R_signals = self.builder.get_object("text_P2U_R_signals")
        self.text_rng_seed = self.builder.get_object("text_rng_seed")
        self.text_serial = self.builder.get_object("text_serial")
        self.text_num_free_stream_buf = self.builder.get_object("text_num_free_stream_buf")

        # affect
        self.text_emotion = self.builder.get_object("text_emotion")
        self.text_mood = self.builder.get_object("text_mood")
        self.text_sleep = self.builder.get_object("text_sleep")
        self.builder.get_object("button_affect_1").connect("clicked", self.button_affect)
        self.builder.get_object("button_affect_2").connect("clicked", self.button_affect)
        self.builder.get_object("button_affect_3").connect("clicked", self.button_affect)
        self.builder.get_object("button_affect_4").connect("clicked", self.button_affect)
        self.builder.get_object("button_affect_5").connect("clicked", self.button_affect)
        self.builder.get_object("button_affect_6").connect("clicked", self.button_affect)
        self.builder.get_object("button_affect_7").connect("clicked", self.button_affect)
        self.builder.get_object("button_affect_8").connect("clicked", self.button_affect)
        self.builder.get_object("button_affect_9").connect("clicked", self.button_affect)

        # config
        self.demo_mode_off = []
        self.demo_mode_on = []
        self.chk_branch_enable = self.chk_config("chk_branch_enable", True)
        self.chk_affect_enable = self.chk_config("chk_affect_enable", True)
        self.chk_affect_adjust_rtc = self.chk_config("chk_affect_adjust_rtc", True)
        self.chk_affect_valence_dynamics = self.chk_config("chk_affect_valence_dynamics", True)
        self.chk_affect_arousal_dynamics = self.chk_config("chk_affect_arousal_dynamics", True)
        self.chk_affect_enable_sleep = self.chk_config("chk_affect_enable_sleep", True)
        self.chk_affect_from_clock = self.chk_config("chk_affect_from_clock", True)
        self.chk_affect_from_wakefulness = self.chk_config("chk_affect_from_wakefulness", True)
        self.chk_affect_from_touch = self.chk_config("chk_affect_from_touch", True)
        self.chk_affect_from_light = self.chk_config("chk_affect_from_light", True)
        self.chk_affect_from_sound = self.chk_config("chk_affect_from_sound", True)
        self.chk_affect_from_accel = self.chk_config("chk_affect_from_accel", True)
        self.chk_affect_from_sleep_blocked = self.chk_config("chk_affect_from_sleep_blocked", False)
        self.chk_affect_randomize_valence = self.chk_config("chk_affect_randomize_valence", False)
        self.chk_affect_fast_sleep_dynamics = self.chk_config("chk_affect_fast_sleep_dynamics", False)
        self.chk_express_enable = self.chk_config("chk_express_enable", True)
        self.chk_express_through_light = self.chk_config("chk_express_through_light", True)
        self.chk_express_through_tail = self.chk_config("chk_express_through_tail", True)
        self.chk_express_through_eyelids = self.chk_config("chk_express_through_eyelids", True)
        self.chk_express_through_ears = self.chk_config("chk_express_through_ears", True)
        self.chk_express_through_vocal = self.chk_config("chk_express_through_vocal", True)
        self.chk_express_through_body = self.chk_config("chk_express_through_body", True)
        self.chk_express_through_ping = self.chk_config("chk_express_through_ping", False)
        self.chk_express_no_pirate_noises = self.chk_config("chk_express_no_pirate_noises")
        self.chk_express_do_pirate_noises = self.chk_config("chk_express_do_pirate_noises")
        self.chk_action_enable = self.chk_config("chk_action_enable", True)
        self.chk_action_debug = self.chk_config("chk_action_debug", False)
        self.chk_action_force_mull = self.chk_config("chk_action_force_mull")
        self.chk_action_randomize_orient = self.chk_config("chk_action_randomize_orient")
        self.chk_action_disable_halt = self.chk_config("chk_action_disable_halt")
        self.chk_action_modulate_by_sonar = self.chk_config("chk_action_modulate_by_sonar", True)
        self.chk_body_enable = self.chk_config("chk_body_enable", True)
        self.chk_body_reset_kc_integrators = self.chk_config("chk_body_reset_kc_integrators")
        self.chk_body_no_push = self.chk_config("chk_body_no_push")
        self.chk_body_no_push_motion = self.chk_config("chk_body_no_push_motion")
        self.chk_body_no_push_translation = self.chk_config("chk_body_no_push_translation")
        self.chk_body_no_push_into_sonar = self.chk_config("chk_body_no_push_into_sonar", True)
        self.chk_enable_pos_control = self.chk_config("chk_enable_pos_control")
        self.chk_enable_cliff_reflex = self.chk_config("chk_enable_cliff_reflex", True)
        self.chk_spatial_enable = self.chk_config("chk_spatial_enable", True)
        self.chk_spatial_ignore_audio = self.chk_config("chk_spatial_ignore_audio")
        self.chk_spatial_ignore_video = self.chk_config("chk_spatial_ignore_video")
        self.chk_spatial_send_priority = self.chk_config("chk_spatial_send_priority")
        self.chk_spatial_send_other = self.chk_config("chk_spatial_send_other")
        self.chk_spatial_no_reaff_compromise = self.chk_config("chk_spatial_no_reaff_compromise")
        self.chk_spatial_no_suppress = self.chk_config("chk_spatial_no_suppress")
        self.chk_spatial_show_compromise = self.chk_config("chk_spatial_show_compromise")
        self.chk_spatial_show_test_pattern = self.chk_config("chk_spatial_show_test_pattern")
        self.chk_test_alarm = self.chk_config("chk_test_alarm")
        self.builder.get_object("button_config_demo").connect("clicked", self.button_config_demo)
        self.builder.get_object("button_config_reset").connect("clicked", self.button_config_reset)

        # bridge
        self.chk_no_publish_mics = self.builder.get_object("chk_no_publish_mics")
        self.chk_no_publish_cams = self.builder.get_object("chk_no_publish_cams")
        self.builder.get_object("button_send_bridge_config").connect("clicked", self.button_send_bridge_config)
        self.adj_sound_index_P3 = self.builder.get_object("adj_sound_index_P3")
        self.spin_sound_index_P3 = self.builder.get_object("spin_sound_index_P3")
        self.builder.get_object("button_send_bridge_stream").connect("clicked", self.button_send_bridge_stream)
        self.combo_frame_size = self.builder.get_object("combo_frame_size")
        self.combo_frame_rate = self.builder.get_object("combo_frame_rate")
        self.builder.get_object("button_send_platform_config").connect("clicked", self.button_send_platform_config)
        self.builder.get_object("button_platform_reset").connect("clicked", self.button_platform_reset)

        # body velocity
        self.button_vel_l = self.builder.get_object("button_vel_l")
        self.button_vel_r = self.builder.get_object("button_vel_r")
        self.button_vel_f = self.builder.get_object("button_vel_f")
        self.button_vel_b = self.builder.get_object("button_vel_b")

        # kinematic chain
        self.chk_drive_kc = self.builder.get_object("chk_drive_kc")
        self.scale_lift = self.builder.get_object("scale_lift")
        self.scale_yaw = self.builder.get_object("scale_yaw")
        self.scale_pitch = self.builder.get_object("scale_pitch")
        self.builder.get_object("button_kc_reset").connect("clicked", self.button_kc_reset)

        # initialise KC
        self.ini_pitch = 100 * (miro.MIRO_PITCH_INI_RAD - miro.MIRO_PITCH_MIN_RAD) / (miro.MIRO_PITCH_MAX_RAD - miro.MIRO_PITCH_MIN_RAD)
        self.scale_pitch.set_value(self.ini_pitch)

        # cosmetic DOFs
        self.chk_drive_cosmetic = self.builder.get_object("chk_drive_cosmetic")
        self.scale_eyelids = self.builder.get_object("scale_eyelids")
        self.scale_tail = self.builder.get_object("scale_tail")
        self.scale_ears_0 = self.builder.get_object("scale_ears_0")
        self.scale_ears_1 = self.builder.get_object("scale_ears_1")
        self.builder.get_object("button_cosmetic_reset").connect("clicked", self.button_cosmetic_reset)
        self.builder.get_object("button_cosmetic_blink").connect("clicked", self.button_cosmetic_blink)

        # currently not offered...
        #self.builder.get_object("button_sound_P1").connect("clicked", self.button_sound_P1)
        #self.builder.get_object("button_sound_P2").connect("clicked", self.button_sound_P2)

        # spatial
        self.image_pril = self.builder.get_object("image_pril")
        self.image_prir = self.builder.get_object("image_prir")
        self.image_priw = self.builder.get_object("image_priw")
        self.text_pp_0 = self.builder.get_object("text_pp_0")
        self.text_pp_1 = self.builder.get_object("text_pp_1")
        self.text_pp_2 = self.builder.get_object("text_pp_2")
        self.builder.get_object("button_spatial_record_start").connect("clicked", self.button_spatial_record_start)
        self.builder.get_object("button_spatial_record_stop").connect("clicked", self.button_spatial_record_stop)

        # body push
        self.button_push_l = self.builder.get_object("button_push_l")
        self.button_push_r = self.builder.get_object("button_push_r")
        self.button_push_u = self.builder.get_object("button_push_u")
        self.button_push_d = self.builder.get_object("button_push_d")
        self.button_push_f = self.builder.get_object("button_push_f")
        self.button_push_b = self.builder.get_object("button_push_b")

        # options
        self.opt = lambda:0
        self.opt.robot_is_phys = None
        self.opt.robot_name = ""
        self.opt.uncompressed = False

        # handle args
        for arg in sys.argv[1:]:
            f = arg.find('=')
            if f == -1:
                key = arg
                val = ""
            else:
                key = arg[:f]
                val = arg[f+1:]
            if key == "robot":
                self.opt.robot_name = val
            elif key == "rob":
                self.opt.robot_is_phys = True
            elif key == "sim":
                self.opt.robot_is_phys = False
            elif key == "uncompressed":
                self.opt.uncompressed = True
            else:
                error("argument not recognised \"" + arg + "\"")

        # check we got at least one
        if len(self.opt.robot_name) == 0:
            error("argument \"robot\" must be specified")

        # infer rob/sim
        if self.opt.robot_is_phys is None:
            if self.opt.robot_name[:3] == "rob":
                self.opt.robot_is_phys = True
                print("inferred that robot server is a physical robot from its name")
            else:
                self.opt.robot_is_phys = False
                print("inferred that robot server is a simulated robot from its name")

        # default data
        self.platform_sensors = None
        self.platform_state = None
        self.platform_mics = None
        self.caml_fifo = fifo(self.opt.uncompressed)
        self.camr_fifo = fifo(self.opt.uncompressed)
        self.pril_fifo = fifo(self.opt.uncompressed)
        self.prir_fifo = fifo(self.opt.uncompressed)
        self.priw_fifo = fifo(self.opt.uncompressed)
        self.rgbl_fifo = fifo(self.opt.uncompressed)
        self.rgbr_fifo = fifo(self.opt.uncompressed)
        self.core_state = None
        self.mood = None

        # thread locking because the opencv decoder seems to be
        # less than thread-safe without it
        #
        # nope, still flaky... what's up with this?
        self.lock = threading.Lock()

        # connect events
        self.window_main.connect("destroy", self.on_window_main_destroy)

        # colors
        #self.window_main.modify_bg(Gtk.StateFlags.NORMAL, Gdk.Color(30000, 25000, 25000))

        # load images
        self.image_caml.set_from_file("../../share/media/test_image_320.png")
        self.image_camr.set_from_file("../../share/media/test_image_320.png")
        self.image_pril.set_from_file("../../share/media/test_image_128.png")
        self.image_prir.set_from_file("../../share/media/test_image_128.png")
        self.image_priw.set_from_file("../../share/media/test_image_w.png")

        # no arguments gives usage
        if len(sys.argv) == 1:
            usage()

        # show main window
        self.window_main.show()

        # subscribe
        topic_root = "/miro/" + self.opt.robot_name
        self.sub_sensors = rospy.Subscriber(topic_root + "/platform/sensors",
                platform_sensors, self.callback_platform_sensors)
        self.sub_state = rospy.Subscriber(topic_root + "/platform/state",
                platform_state, self.callback_platform_state)
        self.sub_mics = rospy.Subscriber(topic_root + "/platform/mics",
                platform_mics, self.callback_platform_mics)
        self.sub_core_state = rospy.Subscriber(topic_root + "/core/state",
                core_state, self.callback_core_state)
        if self.opt.uncompressed:
            self.sub_caml = rospy.Subscriber(topic_root + "/platform/caml",
                    Image, self.callback_caml)
            self.sub_camr = rospy.Subscriber(topic_root + "/platform/camr",
                    Image, self.callback_camr)
            self.sub_pril = rospy.Subscriber(topic_root + "/core/pril",
                    Image, self.callback_pril)
            self.sub_prir = rospy.Subscriber(topic_root + "/core/prir",
                    Image, self.callback_prir)
            self.sub_priw = rospy.Subscriber(topic_root + "/core/priw",
                    Image, self.callback_priw)
            self.sub_rgbl = rospy.Subscriber(topic_root + "/core/rgbl",
                    Image, self.callback_rgbl)
            self.sub_rgbr = rospy.Subscriber(topic_root + "/core/rgbr",
                    Image, self.callback_rgbr)
        else:
            self.sub_caml = rospy.Subscriber(topic_root + "/platform/caml/compressed",
                    CompressedImage, self.callback_caml)
            self.sub_camr = rospy.Subscriber(topic_root + "/platform/camr/compressed",
                    CompressedImage, self.callback_camr)
            self.sub_pril = rospy.Subscriber(topic_root + "/core/pril/compressed",
                    CompressedImage, self.callback_pril)
            self.sub_prir = rospy.Subscriber(topic_root + "/core/prir/compressed",
                    CompressedImage, self.callback_prir)
            self.sub_priw = rospy.Subscriber(topic_root + "/core/priw/compressed",
                    CompressedImage, self.callback_priw)
            self.sub_rgbl = rospy.Subscriber(topic_root + "/core/rgbl/compressed",
                    CompressedImage, self.callback_rgbl)
            self.sub_rgbr = rospy.Subscriber(topic_root + "/core/rgbr/compressed",
                    CompressedImage, self.callback_rgbr)

        # publish
        self.pub_platform_control = rospy.Publisher(topic_root + "/platform/control",
                    platform_control, queue_size=0)
        self.pub_core_control = rospy.Publisher(topic_root + "/core/control",
                    core_control, queue_size=0)
        self.pub_core_config = rospy.Publisher(topic_root + "/core/config",
                    core_config, queue_size=0)
        self.pub_bridge_config = rospy.Publisher(topic_root + "/bridge/config",
                    bridge_config, queue_size=0)
        self.pub_bridge_stream = rospy.Publisher(topic_root + "/bridge/stream",
                    bridge_stream, queue_size=0)
        self.pub_platform_config = rospy.Publisher(topic_root + "/platform/config",
                    platform_config, queue_size=0)

        # config
        self.do_config = 50
        self.pulse = 0
        self.blink = 0
        self.sound_index_P1 = 0
        self.sound_index_P2 = 0

        # bridge config
        self.send_bridge_config = False
        self.no_publish_mics = False
        self.no_publish_cams = False

        # bridge stream
        self.send_bridge_stream = False
        self.sound_index_P3 = 0

        # platform config
        self.send_platform_config = False
        self.frame_size = 0
        self.frame_rate = 0
        self.platform_reset = 0

        # mics buffers
        self.w_mics = 320
        self.h_mics = 240
        self.ima = bytearray(self.w_mics*self.h_mics*3)
        self.imb = bytearray(self.w_mics*self.h_mics*3)
        self.spatial_record = 0

        # start update timer
        GLib.timeout_add(100, self.update_ui)

if __name__ == "__main__":
    rospy.init_node("miro_ros_client_gui_py", anonymous=True)
    main = miro_ros_client_gui()
    Gtk.main()



