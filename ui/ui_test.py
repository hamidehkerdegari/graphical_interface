#!/usr/bin/env python
################################################################
import rospy
import interface_lib as lib
import matplotlib
matplotlib.rcParams['toolbar'] = 'None'
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from matplotlib.widgets import Cursor
from matplotlib.widgets import Button, RadioButtons

# ==============================================================

class Cl_Button(object):
    def __init__(self, text, image, color, hovercolor, x, y , callback):
        self.callback = callback

        axButton = plt.axes([x, y, 0.05/1.6, 0.05/1.6])
        for spine in plt.gca().spines.values():  # Get rid of the frame
            spine.set_visible(False)
        #axButton.imshow(Im_Button)
        self.But = Button(axButton, text, image=image, color=color, hovercolor=hovercolor)
        self.But.on_clicked(self.click)

    def click(self, event):
        self.animate_MainWindow = False
        self.callback()
        plt.show()

#==============================

class MiroGI():
    def __init__(self):
        self.show_pri = False
        self.animate_MainWindow = True

        self.interval = 500
        self.screen_size = [800, 450]
        self.opacity = 1.0

        rospy.init_node("miro_ros_client_py", anonymous=True)
        self.miro = lib.miro_ros_client()

        self.img_caml = plt.imread('../documents/caml.png')
        self.img_camr = plt.imread('../documents/camr.png')
        self.img_priw = plt.imread('../documents/priw.jpg')

        self.Main_Window = self.initMainWindow()
        plt.show()

    def RmFrame(self):
        for spine in plt.gca().spines.values():  # Get rid of the frame
            spine.set_visible(False)

    def initMainWindow(self):
        # Initializing the main window.
        fig_main, ax_main = plt.subplots(nrows=1, ncols=1, figsize=(9, 5))
        fig_main.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.0, wspace=0.0, hspace=0.0)
        pltManager = plt.get_current_fig_manager()
        pltManager.full_screen_toggle()
        #pltManager.resize(*pltManager.window.maxsize())  # Make full screen
        fig_main.canvas.set_window_title('MiRo Graphical Interface')

        # Setting the back/static image(s).
        img_back = plt.imread('../documents/biomimetic_core.png')
        ax_main.imshow(img_back, zorder=0, aspect='auto', extent=[0, self.screen_size[0], 0, self.screen_size[1]])
        ax_main.axis('off')  # clear x- and y-axes
        ax_main.set_xlim([0, self.screen_size[0]])
        ax_main.set_ylim([0, self.screen_size[1]])

        # Initializing the GPR plot.
        self.ax_GPR = lib.add_subplot(ax_main, fig_main, [0.82, 0.61, 0.15, 0.12])
        self.index = np.arange(8)
        self.bar_width = 0.7
        self.RmFrame()
        self.ax_GPR.patch.set_visible(False)  # Remove backgrounf
        self.ax_GPR.tick_params(top='off', bottom='off', left='off', right='off', labelleft='off', labelbottom='on')
        self.ax_GPR.set_xticks(self.index + self.bar_width / 2)
        self.ax_GPR.set_xticklabels(['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'])
        self.colors = ['tomato', 'darkcyan', 'paleturquoise', 'blueviolet', 'hotpink', 'seagreen', 'navy']
        self.plt_GPR_handle = self.ax_GPR.bar(self.index, (1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0), self.bar_width, zorder=1, alpha=self.opacity, color=self.colors)

        #  Initializing moving circle.
        self.ax_circle = lib.add_subplot(ax_main, fig_main, [0.615, 0.375, 0.3 * 9.0 / 16.0, 0.3])
        self.RmFrame()
        self.ax_circle.patch.set_visible(False)  # Remove backgrounf
        self.ax_circle.tick_params(top='off', bottom='off', left='off', right='off', labelleft='off', labelbottom='off')
        self.ax_circle.set_xlim([-10, 10])
        self.ax_circle.set_ylim([-10, 10])
        self.ax_circle.set_aspect('auto')
        self.plt_circle_red_handle = self.ax_circle.scatter(0, 0, s=200, c='r', alpha=self.opacity, zorder=1)
        self.plt_circle_blue_handle = self.ax_circle.scatter(0, 0, s=200, c='b', alpha=self.opacity, zorder=1)
        self.plt_circle_yellow_handle = self.ax_circle.scatter(0, 0, s=200, c='y', alpha=self.opacity, zorder=1)

        #  Initializing camera left.
        self.ax_camera_l = lib.add_subplot(ax_main, fig_main, [0.294, 0.41, 0.15, 0.17])
        self.RmFrame()
        self.ax_camera_l.patch.set_visible(False)  # Remove backgrounf
        self.ax_camera_l.tick_params(top='off', bottom='off', left='off', right='off', labelleft='off', labelbottom='off')
        self.plt_camera_l_handle = self.ax_camera_l.imshow(self.img_caml, zorder=1, aspect='auto')

        #  Initializing camera right.
        self.ax_camera_r = lib.add_subplot(ax_main, fig_main, [0.384, 0.41, 0.15, 0.17])
        self.RmFrame()
        self.ax_camera_r.patch.set_visible(False)  # Remove backgrounf
        self.ax_camera_r.tick_params(top='off', bottom='off', left='off', right='off', labelleft='off', labelbottom='off')
        self.plt_camera_r_handle = self.ax_camera_r.imshow(self.img_camr, zorder=1, aspect='auto')

        #  Initializing priw.
        self.ax_priw = lib.add_subplot(ax_main, fig_main, [0.2655, 0.602, 0.3, 0.025])
        self.RmFrame()
        self.ax_priw.patch.set_visible(False)  # Remove backgrounf
        self.ax_priw.tick_params(top='off', bottom='off', left='off', right='off', labelleft='off', labelbottom='off')
        self.plt_priw_handle = self.ax_priw.imshow(self.img_priw, zorder=1, extent=[0, 320, 0, 16])

        #  Initializing priorities.
        self.ax_priorities = lib.add_subplot(ax_main, fig_main, [0.18, 0.78, 0.28, 0.18])
        self.RmFrame()
        self.ax_priorities.patch.set_visible(False)  # Remove backgrounf
        self.ax_priorities.tick_params(top='off', bottom='off', left='off', right='off', labelleft='off', labelbottom='off')
        self.ax_priorities.set_xlim([0, 30])
        self.ax_priorities.set_ylim([0, 10])
        self.ax_priorities.set_aspect('auto')

        self.ax_priorities.scatter(3.8, 5.8, s=1600, c='w', linewidths=1, edgecolor='k', alpha=1, zorder=0)
        self.plt_priority_1_handle = self.ax_priorities.scatter(3.8, 5.8, s=1600, c='r', linewidths=1, edgecolor='k', alpha=0.5, zorder=1)
        self.ax_priorities.text(3.8, 5.8, "A1", size=20, ha="center", va="center", zorder=2)

        self.ax_priorities.scatter(7.0, 3.0, s=1600, c='w', linewidths=1, edgecolor='k', alpha=1, zorder=0)
        self.plt_priority_2_handle = self.ax_priorities.scatter(7.0, 3.0, s=1600, c='r', linewidths=1, edgecolor='k', alpha=0.5, zorder=1)
        self.ax_priorities.text(7.0, 3.0, "A2", size=20, ha="center", va="center", zorder=2)

        self.ax_priorities.scatter(10.3, 5.8, s=1600, c='w', linewidths=1, edgecolor='k', alpha=1, zorder=0)
        self.plt_priority_3_handle = self.ax_priorities.scatter(10.3, 5.8, s=1600, c='r', linewidths=1, edgecolor='k', alpha=0.5, zorder=1)
        self.ax_priorities.text(10.3, 5.8, "A3", size=20, ha="center", va="center", zorder=2)

        self.ax_priorities.scatter(13.6, 3.0, s=1600, c='w', linewidths=1, edgecolor='k', alpha=1, zorder=0)
        self.plt_priority_4_handle = self.ax_priorities.scatter(13.6, 3.0, s=1600, c='r', linewidths=1, edgecolor='k', alpha=0.5, zorder=1)
        self.ax_priorities.text(13.6, 3.0, "A4", size=20, ha="center", va="center", zorder=2)

        self.ax_priorities.scatter(16.8, 5.8, s=1600, c='w', linewidths=1, edgecolor='k', alpha=1, zorder=0)
        self.plt_priority_5_handle = self.ax_priorities.scatter(16.8, 5.8, s=1600, c='r', linewidths=1, edgecolor='k', alpha=0.5, zorder=1)
        self.ax_priorities.text(16.8, 5.8, "A5", size=20, ha="center", va="center", zorder=2)

        self.ax_priorities.scatter(20.1, 3.0, s=1600, c='w', linewidths=1, edgecolor='k', alpha=1, zorder=0)
        self.plt_priority_6_handle = self.ax_priorities.scatter(20.1, 3.0, s=1600, c='r', linewidths=1, edgecolor='k', alpha=0.5, zorder=1)
        self.ax_priorities.text(20.1, 3.0, "A6", size=20, ha="center", va="center", zorder=2)

        self.ax_priorities.scatter(23.3, 5.8, s=1600, c='w', linewidths=1, edgecolor='k', alpha=1, zorder=0)
        self.plt_priority_7_handle = self.ax_priorities.scatter(23.3, 5.8, s=1600, c='r', linewidths=1, edgecolor='k', alpha=0.5, zorder=1)
        self.ax_priorities.text(23.3, 5.8, "A7", size=20, ha="center", va="center", zorder=2)

        self.ax_priorities.scatter(26.6, 3.0, s=1600, c='w', linewidths=1, edgecolor='k', alpha=1, zorder=0)
        self.plt_priority_8_handle = self.ax_priorities.scatter(26.6, 3.0, s=1600, c='r', linewidths=1, edgecolor='k', alpha=0.5, zorder=1)
        self.ax_priorities.text(26.6, 3.0, "A8", size=20, ha="center", va="center", zorder=2)

        # Initialize Biological Clock time.
        ang = np.deg2rad(45)
        self.ax_bioclock = lib.add_subplot(ax_main, fig_main, [0.523, 0.068, 0.24 * 9.0 / 16.0, 0.24])
        self.RmFrame()
        self.ax_bioclock.patch.set_visible(False)  # Remove backgrounf
        self.ax_bioclock.tick_params(top='off', bottom='off', left='off', right='off', labelleft='off', labelbottom='off')
        self.ax_bioclock.set_xlim([-1, 1])
        self.ax_bioclock.set_ylim([-1, 1])
        self.ax_bioclock.set_aspect('auto')
        self.ax_bioclock_handle = self.ax_bioclock.arrow(0, 0, np.cos(ang) * 0.7, np.sin(ang) * 0.7, head_width=0.05, head_length=0.1, fc='k', ec='k')

        # cursor = Cursor()
        cid = fig_main.canvas.mpl_connect('button_press_event', self.handle_click)

        # Initialize Buttons.
        Im_Button = plt.imread('../documents/full_screen.png')
        self.ButAM = Cl_Button('', Im_Button, 'honeydew', 'w', 0.542, 0.38, self.initSpetialAMWindow)
        self.ButGPR = Cl_Button('', Im_Button, 'w', 'whitesmoke', 0.654, 0.762, self.initGPRWindow)
        self.ButAS = Cl_Button('', Im_Button, 'w', 'whitesmoke', 0.755, 0.38, self.initAffectStateWindow)
        Im_Close = plt.imread('../documents/close.png')
        self.ButAS = Cl_Button('', Im_Close, 'whitesmoke', 'paleturquoise', 0.0, 0.96, plt.close)

        return animation.FuncAnimation(fig_main, self.MainWindow_Updatefig, interval=self.interval)

    #=========================

    def initSpetialAMWindow(self):
        # Initializing a new window.
        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(9, 5))
        fig.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.0, wspace=0.0, hspace=0.0)
        pltManager = plt.get_current_fig_manager()
        pltManager.full_screen_toggle()
        #pltManager.resize(*pltManager.window.maxsize())  # Make full screen
        fig.canvas.set_window_title('Spatial Attention Model')

        # Setting the back/static image(s).
        img_back = plt.imread('../documents/SpetialAM.png')
        ax.imshow(img_back, zorder=0, aspect='auto', extent=[0, self.screen_size[0], 0, self.screen_size[1]])
        ax.axis('off')  # clear x- and y-axes
        ax.set_xlim([0, self.screen_size[0]])
        ax.set_ylim([0, self.screen_size[1]])

        #  Initializing camera left.
        self.ax_camera_l_SAM = lib.add_subplot(ax, fig, [0.294, 0.41, 0.15, 0.17])
        self.RmFrame()
        self.ax_camera_l_SAM.patch.set_visible(False)  # Remove backgrounf
        self.ax_camera_l_SAM.tick_params(top='off', bottom='off', left='off', right='off', labelleft='off',
                                     labelbottom='off')
        self.plt_camera_l_handle_SAM = self.ax_camera_l_SAM.imshow(self.img_caml, zorder=1, aspect='auto')

        #  Initializing camera right.
        self.ax_camera_r_SAM = lib.add_subplot(ax, fig, [0.384, 0.41, 0.15, 0.17])
        self.RmFrame()
        self.ax_camera_r_SAM.patch.set_visible(False)  # Remove backgrounf
        self.ax_camera_r_SAM.tick_params(top='off', bottom='off', left='off', right='off', labelleft='off',
                                     labelbottom='off')
        self.plt_camera_r_handle_SAM = self.ax_camera_r_SAM.imshow(self.img_camr, zorder=1, aspect='auto')

        #  Initializing priw.
        self.ax_priw_SAM = lib.add_subplot(ax, fig, [0.2655, 0.602, 0.3, 0.025])
        self.RmFrame()
        self.ax_priw_SAM.patch.set_visible(False)  # Remove backgrounf
        self.ax_priw_SAM.tick_params(top='off', bottom='off', left='off', right='off', labelleft='off', labelbottom='off')
        self.plt_priw_handle_SAM = self.ax_priw_SAM.imshow(self.img_priw, zorder=1, extent=[0, 320, 0, 16])

        Im_Back = plt.imread('../documents/back.png')
        self.ButAS = Cl_Button('', Im_Back, 'whitesmoke', 'paleturquoise', 0.0, 0.96, plt.close)

        cid = fig.canvas.mpl_connect('close_event', self.handle_close)

        return animation.FuncAnimation(fig, self.SpetialAMWindow_Updatefig, interval=self.interval)

    # =========================

    def initGPRWindow(self):
        # Initializing a new window.
        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(9, 5))
        fig.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.0, wspace=0.0, hspace=0.0)
        pltManager = plt.get_current_fig_manager()
        pltManager.full_screen_toggle()
        #pltManager.resize(*pltManager.window.maxsize())  # Make full screen
        fig.canvas.set_window_title('GPR Basal Ganglia Model (branch s/w)')

        # Setting the back/static image.
        img_back = plt.imread('../documents/GPRWindow.png')
        ax.imshow(img_back, zorder=0, aspect='auto', extent=[0, self.screen_size[0], 0, self.screen_size[1]])
        ax.axis('off')  # clear x- and y-axes
        ax.set_xlim([0, self.screen_size[0]])
        ax.set_ylim([0, self.screen_size[1]])

        # Initializing the GPR plot.
        self.ax_GPRWin = lib.add_subplot(ax, fig, [0.82, 0.61, 0.15, 0.12])
        self.RmFrame()
        self.ax_GPRWin.patch.set_visible(False)  # Remove backgrounf
        self.ax_GPRWin.tick_params(top='off', bottom='off', left='off', right='off', labelleft='off', labelbottom='on')
        self.ax_GPRWin.set_xticks(self.index + self.bar_width / 2)
        self.ax_GPRWin.set_xticklabels(['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'])
        self.plt_GPRWin_handle = self.ax_GPRWin.bar(self.index, (1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0), self.bar_width, zorder=1, alpha=self.opacity, color=self.colors)

        Im_Back = plt.imread('../documents/back.png')
        self.ButAS = Cl_Button('', Im_Back, 'whitesmoke', 'paleturquoise', 0.0, 0.96, plt.close)

        cid = fig.canvas.mpl_connect('close_event', self.handle_close)

        return animation.FuncAnimation(fig, self.GPRWindow_Updatefig, interval=self.interval)

    # =========================

    def initAffectStateWindow(self):
        # Initializing a new window.
        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(9, 5))
        fig.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.0, wspace=0.0, hspace=0.0)
        pltManager = plt.get_current_fig_manager()
        pltManager.full_screen_toggle()
        #pltManager.resize(*pltManager.window.maxsize())  # Make full screen
        fig.canvas.set_window_title('Affect & states & dynamics')

        # Setting the back/static image.
        img_back = plt.imread('../documents/affect_state.png')
        ax.imshow(img_back, zorder=0, aspect='auto', extent=[0, self.screen_size[0], 0, self.screen_size[1]])
        ax.axis('off')  # clear x- and y-axes
        ax.set_xlim([0, self.screen_size[0]])
        ax.set_ylim([0, self.screen_size[1]])

        #  Initializing moving circles.
        self.ax_circle_AS = lib.add_subplot(ax, fig, [0.303, 0.105, 0.7 * 9.0 / 16.0, 0.7])
        self.RmFrame()
        self.ax_circle_AS.patch.set_visible(False)  # Remove backgroun
        self.ax_circle_AS.tick_params(top='off', bottom='off', left='off', right='off', labelleft='off', labelbottom='off')
        self.ax_circle_AS.set_xlim([-10, 10])
        self.ax_circle_AS.set_ylim([-10, 10])
        self.ax_circle_AS.set_aspect('auto')
        self.plt_circle_red_handle_AS = self.ax_circle_AS.scatter(0, 0, s=200, c='r', alpha=self.opacity, zorder=1)
        self.plt_circle_blue_handle_AS = self.ax_circle_AS.scatter(0, 0, s=200, c='b', alpha=self.opacity, zorder=1)
        self.plt_circle_yellow_handle_AS = self.ax_circle_AS.scatter(0, 0, s=200, c='y', alpha=self.opacity, zorder=1)

        Im_Back = plt.imread('../documents/back.png')
        self.ButAS = Cl_Button('', Im_Back, 'whitesmoke', 'paleturquoise', 0.0, 0.96, plt.close)

        cid = fig.canvas.mpl_connect('close_event', self.handle_close)

        return animation.FuncAnimation(fig, self.AffectStateWindow_Updatefig, interval=self.interval)

    # =========================

    def MainWindow_Updatefig(self, i):
        if rospy.core.is_shutdown() or not self.animate_MainWindow:
            return

        print 'MainWindow_Updatefig'
        #print 'H:', self.miro.rtc_hrs, 'M:',self.miro.rtc_mins, 'S:',self.miro.rtc_secs, 'skew:',self.miro.rtc_skew

        if (self.miro.image_priw is not None):
            self.plt_priw_handle.remove()
            self.plt_priw_handle = self.ax_priw.imshow(self.miro.image_priw[:, :, 0], zorder=1, extent=[0, 320, 0, 16], interpolation='none', cmap='jet')

        if self.show_pri:
            if (self.miro.image_pril is not None) and (self.miro.image_prir is not None):
                self.plt_camera_l_handle.remove()
                self.plt_camera_l_handle = self.ax_camera_l.imshow(self.miro.image_pril[:, :, 0], zorder=1, alpha=1, aspect='auto', interpolation='gaussian', cmap='jet')

                self.plt_camera_r_handle.remove()
                self.plt_camera_r_handle = self.ax_camera_r.imshow(self.miro.image_prir[:, :, 0], zorder=1, alpha=1, aspect='auto', interpolation='gaussian', cmap='jet')
        else:
            if (self.miro.image_caml is not None) and (self.miro.image_camr is not None):
                self.plt_camera_l_handle.remove()
                self.plt_camera_l_handle = self.ax_camera_l.imshow(self.miro.image_caml, zorder=1, aspect='auto')

                self.plt_camera_r_handle.remove()
                self.plt_camera_r_handle = self.ax_camera_r.imshow(self.miro.image_camr, zorder=1, aspect='auto')


        if (self.miro.platform_sensors is not None) and (self.miro.core_state is not None):
            self.miro.update_data()

            # Plotting animation
            self.plt_circle_red_handle.remove()
            self.plt_circle_blue_handle.remove()
            self.plt_circle_yellow_handle.remove()
            x = self.miro.accel_head.x
            y = self.miro.accel_head.y
            self.plt_circle_red_handle = self.ax_circle.scatter(self.miro.emotion.valence*16.0-8.0, self.miro.emotion.arousal*16.0-8.0, s=200, c='r', alpha=self.opacity, zorder=1)
            self.plt_circle_blue_handle = self.ax_circle.scatter(self.miro.mood.valence*16.0-8.0, self.miro.mood.arousal*16.0-8.0, s=200, c='b', alpha=self.opacity, zorder=1)
            self.plt_circle_yellow_handle = self.ax_circle.scatter(self.miro.sleep.wakefulness*16.0-8.0, self.miro.sleep.pressure*16.0-8.0, s=200, c='y', alpha=self.opacity, zorder=1)

            self.plt_GPR_handle.remove()
            self.plt_GPR_handle = self.ax_GPR.bar(self.index, self.miro.selection, self.bar_width, zorder=1, alpha=self.opacity, color=self.colors)

            p = self.miro.priority
            d = self.miro.disinhibition
            self.plt_priority_1_handle.remove()
            self.plt_priority_2_handle.remove()
            self.plt_priority_3_handle.remove()
            self.plt_priority_4_handle.remove()
            self.plt_priority_5_handle.remove()
            self.plt_priority_6_handle.remove()
            self.plt_priority_7_handle.remove()
            self.plt_priority_8_handle.remove()
            self.plt_priority_1_handle = self.ax_priorities.scatter(3.80, 5.8, s=1600, c='r', linewidths=3*d[0], edgecolor='k', alpha=p[0], zorder=1)
            self.plt_priority_2_handle = self.ax_priorities.scatter(7.00, 3.0, s=1600, c='r', linewidths=3*d[1], edgecolor='k', alpha=p[1], zorder=1)
            self.plt_priority_3_handle = self.ax_priorities.scatter(10.3, 5.8, s=1600, c='r', linewidths=3*d[2], edgecolor='k', alpha=p[2], zorder=1)
            self.plt_priority_4_handle = self.ax_priorities.scatter(13.6, 3.0, s=1600, c='r', linewidths=3*d[3], edgecolor='k', alpha=p[3], zorder=1)
            self.plt_priority_5_handle = self.ax_priorities.scatter(16.8, 5.8, s=1600, c='r', linewidths=3*d[4], edgecolor='k', alpha=p[4], zorder=1)
            self.plt_priority_6_handle = self.ax_priorities.scatter(20.1, 3.0, s=1600, c='r', linewidths=3*d[5], edgecolor='k', alpha=p[5], zorder=1)
            self.plt_priority_7_handle = self.ax_priorities.scatter(23.3, 5.8, s=1600, c='r', linewidths=3*d[6], edgecolor='k', alpha=p[6], zorder=1)
            self.plt_priority_8_handle = self.ax_priorities.scatter(26.6, 3.0, s=1600, c='r', linewidths=3*d[7], edgecolor='k', alpha=p[7], zorder=1)

            # Initialize Biological Clock time.
            self.ax_bioclock_handle.remove()
            ang = np.deg2rad(45)
            self.ax_bioclock_handle = self.ax_bioclock.arrow(0, 0, np.cos(ang) * 0.7, np.sin(ang) * 0.7, head_width=0.05, head_length=0.1, fc='k', ec='k')

    # =========================

    def SpetialAMWindow_Updatefig(self, i):
        if rospy.core.is_shutdown():
            return

        print 'SpetialAMWindow_Updatefig'

        if (self.miro.image_priw is not None):
            self.plt_priw_handle_SAM.remove()
            self.plt_priw_handle_SAM = self.ax_priw_SAM.imshow(self.miro.image_priw[:, :, 0], zorder=1, extent=[0, 320, 0, 16], interpolation='none', cmap='jet')

        if self.show_pri:
            if (self.miro.image_pril is not None) and (self.miro.image_prir is not None):
                self.plt_camera_l_handle_SAM.remove()
                self.plt_camera_l_handle_SAM = self.ax_camera_l_SAM.imshow(self.miro.image_pril[:, :, 0], zorder=1, alpha=1, aspect='auto', interpolation='gaussian', cmap='jet')

                self.plt_camera_r_handle_SAM.remove()
                self.plt_camera_r_handle_SAM = self.ax_camera_r_SAM.imshow(self.miro.image_prir[:, :, 0], zorder=1, alpha=1, aspect='auto', interpolation='gaussian', cmap='jet')
        else:
            if (self.miro.image_caml is not None) and (self.miro.image_camr is not None):
                self.plt_camera_l_handle_SAM.remove()
                self.plt_camera_l_handle_SAM = self.ax_camera_l_SAM.imshow(self.miro.image_caml, zorder=1, aspect='auto')

                self.plt_camera_r_handle_SAM.remove()
                self.plt_camera_r_handle_SAM = self.ax_camera_r_SAM.imshow(self.miro.image_camr, zorder=1, aspect='auto')

    # =========================

    def GPRWindow_Updatefig(self, i):
        if rospy.core.is_shutdown():
            return

        if (self.miro.platform_sensors is not None) and (self.miro.core_state is not None):
            self.miro.update_data()

            self.plt_GPRWin_handle.remove()
            self.plt_GPRWin_handle = self.ax_GPRWin.bar(self.index, self.miro.selection, self.bar_width, zorder=1, alpha=self.opacity, color=self.colors)

    # =========================

    def AffectStateWindow_Updatefig(self, i):
        if rospy.core.is_shutdown():
            return

        if (self.miro.platform_sensors is not None) and (self.miro.core_state is not None):
            self.miro.update_data()

            # Plotting animation
            self.plt_circle_red_handle_AS.remove()
            self.plt_circle_blue_handle_AS.remove()
            self.plt_circle_yellow_handle_AS.remove()
            self.plt_circle_red_handle_AS = self.ax_circle_AS.scatter(self.miro.emotion.valence * 16.0 - 8.0,
                                                                self.miro.emotion.arousal * 16.0 - 8.0, s=200, c='r',
                                                                alpha=self.opacity, zorder=1)
            self.plt_circle_blue_handle_AS = self.ax_circle_AS.scatter(self.miro.mood.valence * 16.0 - 8.0,
                                                                 self.miro.mood.arousal * 16.0 - 8.0, s=200, c='b',
                                                                 alpha=self.opacity, zorder=1)
            self.plt_circle_yellow_handle_AS = self.ax_circle_AS.scatter(self.miro.sleep.wakefulness * 16.0 - 8.0,
                                                                   self.miro.sleep.pressure * 16.0 - 8.0, s=200, c='y',
                                                                   alpha=self.opacity, zorder=1)

    # =========================

    # Getting the cursor click position.
    def handle_click(self, event):
        print('button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
              (event.button, event.x, event.y, event.xdata, event.ydata))

        if 383 < event.x and event.x < 694 and 280 < event.y and event.y < 396:
            # OnClick for switching between cameras and pri
            self.show_pri = not self.show_pri

    def handle_close(self, event):
        print('Figure closed!')
        self.animate_MainWindow = True

################################################################
if __name__ == "__main__":
    # Setting the back/static image(s).
    MainGI = MiroGI()

################################################################
