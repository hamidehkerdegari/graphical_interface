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
import matplotlib.patches as patches
from scipy.ndimage.filters import gaussian_filter

# ==============================================================
global animate_MainWindow
animate_MainWindow = True

UpdateRate = 20  # Updating Rate in Hz

def RmFrame():
    for spine in plt.gca().spines.values():  # Get rid of the frame
        spine.set_visible(False)

class Cl_Button(object):
    def __init__(self, text, image, color, hovercolor, x, y, callback):
        self.callback = callback

        self.axButton = plt.axes([x, y, 0.05/1.6, 0.05/1.6], zorder=2)
        RmFrame()
        #axButton.imshow(Im_Button)
        self.But = Button(self.axButton, text, image=image, color=color, hovercolor=hovercolor)
        self.But.on_clicked(self.click)

    def click(self, event):
        global animate_MainWindow
        animate_MainWindow = False
        self.callback()
        plt.show()

#==============================

class MiroGI():
    def __init__(self):
        self.interval = 1000/UpdateRate
        self.screen_size = [800, 450]
        self.opacity = 1.0

        rospy.init_node("miro_ros_client_py", anonymous=True)
        self.miro = lib.miro_ros_client()

        self.img_caml = plt.imread('../documents/caml.png')
        self.img_camr = plt.imread('../documents/camr.png')
        self.img_priw = plt.imread('../documents/priw.jpg')

        self.img_back_main = plt.imread('../documents/biomimetic_core.png')
        self.img_back_SAM = plt.imread('../documents/SpetialAM.png')
        self.img_back_GPR = plt.imread('../documents/GPRWindow.png')
        self.img_back_AS = plt.imread('../documents/affect_state.png')
        self.img_back_InOut = plt.imread('../documents/in_out.png')

        self.init_MainWindow()
        plt.show()

    def init_MainWindow(self):
        # Initializing the main window.
        fig_main, ax_main = plt.subplots(nrows=1, ncols=1, figsize=(9, 5))
        fig_main.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.0, wspace=0.0, hspace=0.0)
        pltManager = plt.get_current_fig_manager()
        #pltManager.full_screen_toggle()
        pltManager.resize(*pltManager.window.maxsize())  # Make full screen
        fig_main.canvas.set_window_title('MiRo Graphical Interface')
        fig_main.canvas.draw()

        # Setting the back/static image(s).
        ax_main.imshow(self.img_back_main, zorder=0, aspect='auto', extent=[0, self.screen_size[0], 0, self.screen_size[1]])
        ax_main.axis('off')  # clear x- and y-axes
        ax_main.set_xlim([0, self.screen_size[0]])
        ax_main.set_ylim([0, self.screen_size[1]])

        # Initializing the GPR plot.
        self.ax_GPR = lib.add_subplot(ax_main, fig_main, [0.82, 0.61, 0.15, 0.12])
        self.index = np.arange(8)
        self.bar_width = 0.7
        RmFrame()
        self.ax_GPR.patch.set_visible(False)  # Remove backgrounf
        self.ax_GPR.tick_params(top='off', bottom='off', left='off', right='off', labelleft='off', labelbottom='on')
        self.ax_GPR.set_xticks(self.index + self.bar_width / 2)
        self.ax_GPR.set_xticklabels(['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'])
        self.colors = ['firebrick', 'green', 'mediumblue', 'm', 'darkcyan', 'olive', 'darkorange','dimgray']

        self.plt_GPR_handle = self.ax_GPR.bar(self.index, (1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0), self.bar_width, zorder=1, alpha=self.opacity, color=self.colors)

        #  Initializing moving circle.
        self.ax_circle = lib.add_subplot(ax_main, fig_main, [0.615, 0.375, 0.3 * 9.0 / 16.0, 0.3])
        RmFrame()
        self.ax_circle.patch.set_visible(False)  # Remove backgrounf
        self.ax_circle.tick_params(top='off', bottom='off', left='off', right='off', labelleft='off', labelbottom='off')
        self.ax_circle.set_xlim([-10, 10])
        self.ax_circle.set_ylim([-10, 10])
        self.ax_circle.set_aspect('auto')
        self.plt_circle_red_handle = self.ax_circle.scatter(0, 0, s=200, c='r', alpha=self.opacity, zorder=1)
        self.plt_circle_blue_handle = self.ax_circle.scatter(0, 0, s=200, c='b', alpha=self.opacity, zorder=1)
        self.plt_circle_yellow_handle = self.ax_circle.scatter(0, 0, s=200, c='y', alpha=self.opacity, zorder=1)

        #  Initializing camera left.
        #self.ax_camera_l = lib.add_subplot(ax_main, fig_main, [0.294, 0.41, 0.15, 0.17])
        self.ax_camera_l = lib.add_subplot(ax_main, fig_main, [0.265, 0.41, 0.15, 0.17])
        RmFrame()
        self.ax_camera_l.patch.set_visible(False)  # Remove backgrounf
        self.ax_camera_l.tick_params(top='off', bottom='off', left='off', right='off', labelleft='off', labelbottom='off')
        self.plt_camera_l_handle = self.ax_camera_l.imshow(self.img_caml, zorder=1, alpha=1, aspect='auto')


        #  Initializing camera right.
        #self.ax_camera_r = lib.add_subplot(ax_main, fig_main, [0.384, 0.41, 0.15, 0.17])
        self.ax_camera_r = lib.add_subplot(ax_main, fig_main, [0.42, 0.41, 0.15, 0.17])
        RmFrame()
        self.ax_camera_r.patch.set_visible(False)  # Remove backgrounf
        self.ax_camera_r.tick_params(top='off', bottom='off', left='off', right='off', labelleft='off', labelbottom='off')
        self.plt_camera_r_handle = self.ax_camera_r.imshow(self.img_camr, zorder=1, alpha=1, aspect='auto')

        #  Initializing priw.
        self.ax_priw = lib.add_subplot(ax_main, fig_main, [0.2655, 0.602, 0.3, 0.025])
        RmFrame()
        self.ax_priw.patch.set_visible(False)  # Remove backgrounf
        self.ax_priw.tick_params(top='off', bottom='off', left='off', right='off', labelleft='off', labelbottom='off')
        self.plt_priw_handle = self.ax_priw.imshow(self.img_priw, zorder=1, extent=[0, 320, 0, 16])

        #  Initializing priorities.
        self.ax_priorities = lib.add_subplot(ax_main, fig_main, [0.18, 0.78, 0.28, 0.18])
        RmFrame()
        self.ax_priorities.patch.set_visible(False)  # Remove backgrounf
        self.ax_priorities.tick_params(top='off', bottom='off', left='off', right='off', labelleft='off', labelbottom='off')
        self.ax_priorities.set_xlim([0, 30])
        self.ax_priorities.set_ylim([0, 10])
        self.ax_priorities.set_aspect('auto')

        self.plt_priority_1B_handle = self.ax_priorities.scatter(3.8, 5.8, s=1600, c='w', linewidths=1, edgecolor='k', alpha=1, zorder=0)
        self.plt_priority_1_handle = self.ax_priorities.scatter(3.8, 5.8, s=1600, c=self.colors[0], linewidths=1, edgecolor='k', alpha=0.5, zorder=1)
        self.ax_priorities.text(3.8, 5.8, "A1", size=20, ha="center", va="center", zorder=2)

        self.plt_priority_2B_handle = self.ax_priorities.scatter(7.0, 3.0, s=1600, c='w', linewidths=1, edgecolor='k', alpha=1, zorder=0)
        self.plt_priority_2_handle = self.ax_priorities.scatter(7.0, 3.0, s=1600, c=self.colors[1], linewidths=1, edgecolor='k', alpha=0.5, zorder=1)
        self.ax_priorities.text(7.0, 3.0, "A2", size=20, ha="center", va="center", zorder=2)

        self.plt_priority_3B_handle = self.ax_priorities.scatter(10.3, 5.8, s=1600, c='w', linewidths=1, edgecolor='k', alpha=1, zorder=0)
        self.plt_priority_3_handle = self.ax_priorities.scatter(10.3, 5.8, s=1600, c=self.colors[2], linewidths=1, edgecolor='k', alpha=0.5, zorder=1)
        self.ax_priorities.text(10.3, 5.8, "A3", size=20, ha="center", va="center", zorder=2)

        self.plt_priority_4B_handle = self.ax_priorities.scatter(13.6, 3.0, s=1600, c='w', linewidths=1, edgecolor='k', alpha=1, zorder=0)
        self.plt_priority_4_handle = self.ax_priorities.scatter(13.6, 3.0, s=1600, c=self.colors[3], linewidths=1, edgecolor='k', alpha=0.5, zorder=1)
        self.ax_priorities.text(13.6, 3.0, "A4", size=20, ha="center", va="center", zorder=2)

        self.plt_priority_5B_handle = self.ax_priorities.scatter(16.8, 5.8, s=1600, c='w', linewidths=1, edgecolor='k', alpha=1, zorder=0)
        self.plt_priority_5_handle = self.ax_priorities.scatter(16.8, 5.8, s=1600, c=self.colors[4], linewidths=1, edgecolor='k', alpha=0.5, zorder=1)
        self.ax_priorities.text(16.8, 5.8, "A5", size=20, ha="center", va="center", zorder=2)

        self.plt_priority_6B_handle = self.ax_priorities.scatter(20.1, 3.0, s=1600, c='w', linewidths=1, edgecolor='k', alpha=1, zorder=0)
        self.plt_priority_6_handle = self.ax_priorities.scatter(20.1, 3.0, s=1600, c=self.colors[5], linewidths=1, edgecolor='k', alpha=0.5, zorder=1)
        self.ax_priorities.text(20.1, 3.0, "A6", size=20, ha="center", va="center", zorder=2)

        self.plt_priority_7B_handle = self.ax_priorities.scatter(23.3, 5.8, s=1600, c='w', linewidths=1, edgecolor='k', alpha=1, zorder=0)
        self.plt_priority_7_handle = self.ax_priorities.scatter(23.3, 5.8, s=1600, c=self.colors[6], linewidths=1, edgecolor='k', alpha=0.5, zorder=1)
        self.ax_priorities.text(23.3, 5.8, "A7", size=20, ha="center", va="center", zorder=2)

        self.plt_priority_8B_handle = self.ax_priorities.scatter(26.6, 3.0, s=1600, c='w', linewidths=1, edgecolor='k', alpha=1, zorder=0)
        self.plt_priority_8_handle = self.ax_priorities.scatter(26.6, 3.0, s=1600, c=self.colors[7], linewidths=1, edgecolor='k', alpha=0.5, zorder=1)
        self.ax_priorities.text(26.6, 3.0, "A8", size=20, ha="center", va="center", zorder=2)

        # Initialize Biological Clock time.
        ang = np.deg2rad(0)
        self.ax_bioclock = lib.add_subplot(ax_main, fig_main, [0.523, 0.068, 0.24 * 9.0 / 16.0, 0.24])
        RmFrame()
        self.ax_bioclock.patch.set_visible(False)  # Remove backgrounf
        self.ax_bioclock.tick_params(top='off', bottom='off', left='off', right='off', labelleft='off', labelbottom='off')
        self.ax_bioclock.set_xlim([-1, 1])
        self.ax_bioclock.set_ylim([-1, 1])
        self.ax_bioclock.set_aspect('auto')
        #self.ax_bioclock_handle = self.ax_bioclock.arrow(0, 0, np.cos(ang) * 0.7, np.sin(ang) * 0.7, head_width=0.05, head_length=0.1, fc='k', ec='k')
        self.ax_bioclock_handle = patches.FancyArrowPatch((0.0, 0.0), (np.cos(ang) * 0.7, np.sin(ang) * 0.7), arrowstyle='wedge', mutation_scale=10, facecolor='k')
        self.ax_bioclock.add_patch(self.ax_bioclock_handle)

        # Initialize Buttons.
        Im_Button = plt.imread('../documents/full_screen.png')
        self.ButAM = Cl_Button('', Im_Button, 'honeydew', 'w', 0.542, 0.375, self.init_SalienceVisualZoom)
        self.ButGPR = Cl_Button('', Im_Button, 'w', 'whitesmoke', 0.654, 0.762, self.init_BasalGangliaZoom)
        self.ButInOut = Cl_Button('', Im_Button, 'w', 'whitesmoke', 0.68, 0.762, self.init_BasalGanglia_InOutZoom)
        self.ButAS = Cl_Button('', Im_Button, 'w', 'whitesmoke', 0.755, 0.38, self.init_AffectStateWindow)

        # Close Button
        #Im_Close = plt.imread('../documents/close.png')
        #self.ButAS = Cl_Button('', Im_Close, 'whitesmoke', 'paleturquoise', 0.0, 0.96, plt.close)

        self.Main_anim = animation.FuncAnimation(fig_main, self.update_MainWindow, interval=self.interval)

    #=========================

    def init_SalienceVisualZoom(self):
        self.show_pri = 1

        # Initializing a new window.
        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(9, 5))
        fig.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.0, wspace=0.0, hspace=0.0)
        pltManager = plt.get_current_fig_manager()
        #pltManager.full_screen_toggle()
        pltManager.resize(*pltManager.window.maxsize())  # Make full screen
        fig.canvas.set_window_title('Spatial Attention Model')

        # Setting the back/static image(s).
        ax.imshow(self.img_back_SAM, zorder=0, aspect='auto', extent=[0, self.screen_size[0], 0, self.screen_size[1]])
        ax.axis('off')  # clear x- and y-axes
        ax.set_xlim([0, self.screen_size[0]])
        ax.set_ylim([0, self.screen_size[1]])

        #  Initializing camera left.
        self.ax_camera_l_SAM = lib.add_subplot(ax, fig, [0.308-0.064, 0.51, 0.15 * 1.91, 0.17 * 1.91])
        RmFrame()
        self.ax_camera_l_SAM.patch.set_visible(False)  # Remove backgrounf
        self.ax_camera_l_SAM.tick_params(top='off', bottom='off', left='off', right='off', labelleft='off', labelbottom='off')
        self.plt_camera_l_handle_SAM = self.ax_camera_l_SAM.imshow(self.img_caml, zorder=1, aspect='auto')

        #  Initializing camera right.
        self.ax_camera_r_SAM = lib.add_subplot(ax, fig, [0.607-0.064, 0.51, 0.15 * 1.91, 0.17 * 1.91])
        RmFrame()
        self.ax_camera_r_SAM.patch.set_visible(False)  # Remove backgrounf
        self.ax_camera_r_SAM.tick_params(top='off', bottom='off', left='off', right='off', labelleft='off', labelbottom='off')
        self.plt_camera_r_handle_SAM = self.ax_camera_r_SAM.imshow(self.img_camr, zorder=1, aspect='auto')


        #  Initializing priw.
        self.ax_priw_SAM = lib.add_subplot(ax, fig, [0.3-0.064, 0.63-0.22, 0.3*2.0, 0.025*2.0])
        RmFrame()
        self.ax_priw_SAM.patch.set_visible(False)  # Remove backgrounf
        self.ax_priw_SAM.tick_params(top='off', bottom='off', left='off', right='off', labelleft='off', labelbottom='off')
        self.plt_priw_handle_SAM = self.ax_priw_SAM.imshow(self.img_priw, zorder=1, extent=[0, 320, 0, 16])

        #  Initializing PRI left.
        self.ax_PRI_l_SAM = lib.add_subplot(ax, fig, [0.308 - 0.064, 0.27 - 0.19, 0.15 * 1.91, 0.17 * 1.91])
        RmFrame()
        self.ax_PRI_l_SAM.patch.set_visible(False)  # Remove backgrounf
        self.ax_PRI_l_SAM.tick_params(top='off', bottom='off', left='off', right='off', labelleft='off',
                                         labelbottom='off')
        self.plt_PRI_l_handle_SAM = self.ax_PRI_l_SAM.imshow(self.img_caml, zorder=1, aspect='auto')

        #  Initializing PRI right.
        self.ax_PRI_r_SAM = lib.add_subplot(ax, fig, [0.607 - 0.064, 0.27 - 0.19, 0.15 * 1.91, 0.17 * 1.91])
        RmFrame()
        self.ax_PRI_r_SAM.patch.set_visible(False)  # Remove backgrounf
        self.ax_PRI_r_SAM.tick_params(top='off', bottom='off', left='off', right='off', labelleft='off',
                                         labelbottom='off')
        self.plt_PRI_r_handle_SAM = self.ax_PRI_r_SAM.imshow(self.img_camr, zorder=1, aspect='auto')

        # Back Button
        #Im_Back = plt.imread('../documents/back.png')
        #self.ButAS = Cl_Button('', Im_Back, 'whitesmoke', 'paleturquoise', 0.0, 0.96, plt.close)

        # Closing event
        fig.canvas.mpl_connect('close_event', self.callback_WinClose)

        self.SAM_anim = animation.FuncAnimation(fig, self.update_SalienceVisualZoom, interval=self.interval)

    # =========================

    def init_BasalGangliaZoom(self):
        # Initializing a new window.
        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(9, 5))
        fig.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.0, wspace=0.0, hspace=0.0)
        pltManager = plt.get_current_fig_manager()
        #pltManager.full_screen_toggle()
        pltManager.resize(*pltManager.window.maxsize())  # Make full screen
        fig.canvas.set_window_title('GPR Basal Ganglia Model (branch s/w)')

        # Setting the back/static image.
        ax.imshow(self.img_back_GPR, zorder=0, aspect='auto', extent=[0, self.screen_size[0], 0, self.screen_size[1]])
        ax.axis('off')  # clear x- and y-axes
        ax.set_xlim([0, self.screen_size[0]])
        ax.set_ylim([0, self.screen_size[1]])

        # Initializing the GPR plot.
        self.ax_GPRWin = lib.add_subplot(ax, fig, [0.51, 0.16, 0.15*2.0, 0.12*2.0])
        RmFrame()
        self.ax_GPRWin.patch.set_visible(False)  # Remove backgrounf
        self.ax_GPRWin.tick_params(top='off', bottom='off', left='off', right='off', labelleft='off', labelbottom='on')
        self.ax_GPRWin.set_xticks(self.index + self.bar_width / 2)
        self.ax_GPRWin.set_xticklabels(['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'])
        self.plt_GPRWin_handle = self.ax_GPRWin.bar(self.index, (1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0), self.bar_width, zorder=1, alpha=self.opacity, color=self.colors)

        # Back Button
        #Im_Back = plt.imread('../documents/back.png')
        #self.ButAS = Cl_Button('', Im_Back, 'whitesmoke', 'paleturquoise', 0.0, 0.96, plt.close)

        cid = fig.canvas.mpl_connect('close_event', self.callback_WinClose)

        self.GPR_anim = animation.FuncAnimation(fig, self.update_BasalGangliaZoom, interval=self.interval)

    # =========================

    def init_BasalGanglia_InOutZoom(self): # Showing inputs (priority) and the outputs (disinhibition)
        # Initializing a new window.
        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(9, 5))
        fig.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.0, wspace=0.0, hspace=0.0)
        pltManager = plt.get_current_fig_manager()
        #pltManager.full_screen_toggle()
        pltManager.resize(*pltManager.window.maxsize())  # Make full screen
        fig.canvas.set_window_title('inputs (priority) and the outputs (disinhibition)')

        # Setting the back/static image.
        ax.imshow(self.img_back_InOut, zorder=0, aspect='auto', extent=[0, self.screen_size[0], 0, self.screen_size[1]])
        ax.axis('off')  # clear x- and y-axes
        ax.set_xlim([0, self.screen_size[0]])
        ax.set_ylim([0, self.screen_size[1]])

        # Initializing the priority plot.
        self.ax_priority = lib.add_subplot(ax, fig, [0.125, 0.715, 0.15*2.0, 0.11*2.0])
        RmFrame()
        self.ax_priority.patch.set_visible(False)  # Remove backgrounf
        self.ax_priority.tick_params(top='off', bottom='off', left='off', right='off', labelleft='off', labelbottom='on')
        self.ax_priority.set_xticks(self.index + self.bar_width / 2)
        self.ax_priority.set_xticklabels(['A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7', 'A8'])
        self.plt_priority_handle = self.ax_priority.bar(self.index, (1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0), self.bar_width, zorder=1, alpha=self.opacity, color=self.colors)

        # Initializing the disinhibition plot.
        self.ax_disinhibition = lib.add_subplot(ax, fig, [0.603, 0.03, 0.15*2.0, 0.11*2.0])
        RmFrame()
        self.ax_disinhibition.patch.set_visible(False)  # Remove backgrounf
        self.ax_disinhibition.tick_params(top='off', bottom='off', left='off', right='off', labelleft='off', labelbottom='on')
        self.ax_disinhibition.set_xticks(self.index + self.bar_width / 2)
        self.ax_disinhibition.set_xticklabels(['A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7', 'A8'])
        self.plt_disinhibition_handle = self.ax_disinhibition.bar(self.index, (1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0), self.bar_width, zorder=1, alpha=self.opacity, color=self.colors)

        # Back Button
        #Im_Back = plt.imread('../documents/back.png')
        #self.ButAS = Cl_Button('', Im_Back, 'whitesmoke', 'paleturquoise', 0.0, 0.96, plt.close)

        cid = fig.canvas.mpl_connect('close_event', self.callback_WinClose)

        self.InOut_anim = animation.FuncAnimation(fig, self.update_BasalGanglia_InOutZoom, interval=self.interval)

    # =========================

    def init_AffectStateWindow(self):
        # Initializing a new window.
        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(9, 5))
        fig.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.0, wspace=0.0, hspace=0.0)
        pltManager = plt.get_current_fig_manager()
        #pltManager.full_screen_toggle()
        pltManager.resize(*pltManager.window.maxsize())  # Make full screen
        fig.canvas.set_window_title('Affect & states & dynamics')

        # Setting the back/static image.
        ax.imshow(self.img_back_AS, zorder=0, aspect='auto', extent=[0, self.screen_size[0], 0, self.screen_size[1]])
        ax.axis('off')  # clear x- and y-axes
        ax.set_xlim([0, self.screen_size[0]])
        ax.set_ylim([0, self.screen_size[1]])

        #  Initializing Emotion and mood circles.
        self.ax_circle_EM = lib.add_subplot(ax, fig, [0.036, 0.069, 0.7 * 9.0 / 16.0, 0.7])
        #RmFrame()
        self.ax_circle_EM.patch.set_visible(False)  # Remove backgroun
        self.ax_circle_EM.tick_params(top='off', bottom='off', left='off', right='off', labelleft='off', labelbottom='off')
        self.ax_circle_EM.set_xlim([-10, 10])
        self.ax_circle_EM.set_ylim([-10, 10])
        self.ax_circle_EM.set_aspect('auto')
        self.plt_circle_red_handle_AS = self.ax_circle_EM.scatter(0, 0, s=600, c='r', alpha=self.opacity, zorder=1)
        self.plt_circle_blue_handle_AS = self.ax_circle_EM.scatter(0, 0, s=600, c='b', alpha=self.opacity, zorder=1)

        #  Initializing Emotion and mood circles.
        self.ax_circle_Sleep = lib.add_subplot(ax, fig, [0.57, 0.069, 0.7 * 9.0 / 16.0, 0.7])
        # RmFrame()
        self.ax_circle_Sleep.patch.set_visible(False)  # Remove backgroun
        self.ax_circle_Sleep.tick_params(top='off', bottom='off', left='off', right='off', labelleft='off', labelbottom='off')
        self.ax_circle_Sleep.set_xlim([-10, 10])
        self.ax_circle_Sleep.set_ylim([-10, 10])
        self.ax_circle_Sleep.set_aspect('auto')
        self.plt_circle_yellow_handle_AS = self.ax_circle_Sleep.scatter(0, 0, s=600, c='y', alpha=self.opacity, zorder=1)

        # Back Button
        #Im_Back = plt.imread('../documents/back.png')
        #self.ButAS = Cl_Button('', Im_Back, 'whitesmoke', 'paleturquoise', 0.0, 0.96, plt.close)

        cid = fig.canvas.mpl_connect('close_event', self.callback_WinClose)

        self.AS_anim = animation.FuncAnimation(fig, self.update_AffectStateWindow, interval=self.interval)

    # =========================

    def update_MainWindow(self, i):
        if rospy.core.is_shutdown() or not animate_MainWindow:
            return

        priw = self.miro.priw_fifo.latest()
        if (priw is not None):
            self.plt_priw_handle.set_data(priw[:, :, 0])

        caml = self.miro.caml_fifo.latest()
        camr = self.miro.camr_fifo.latest()
        if (caml is not None) and (camr is not None):
            self.plt_camera_l_handle.set_data(caml)
            self.plt_camera_r_handle.set_data(camr)


        if (self.miro.platform_sensors is not None) and (self.miro.core_state is not None):
            # Updating emotion
            self.plt_circle_red_handle.set_offsets([self.miro.core_state.emotion.valence * 16.0 - 8.0, self.miro.core_state.emotion.arousal * 16.0 - 8.0])
            self.plt_circle_blue_handle.set_offsets([self.miro.core_state.mood.valence*16.0-8.0, self.miro.core_state.mood.arousal*16.0-8.0])
            self.plt_circle_yellow_handle.set_offsets([self.miro.core_state.sleep.wakefulness*16.0-8.0, self.miro.core_state.sleep.pressure*16.0-8.0])

            # Updating GPR
            for bar, h in zip(self.plt_GPR_handle, self.miro.core_state.selection):
                bar.set_height(h)

            # Updating priority
            p = self.miro.core_state.priority
            d = self.miro.core_state.disinhibition

            self.plt_priority_1B_handle.set_linewidths(1+3 * d[0])
            self.plt_priority_1_handle.set_linewidths(3 * d[0])
            self.plt_priority_1_handle.set_alpha(p[0])

            self.plt_priority_2B_handle.set_linewidths(1+3 * d[1])
            self.plt_priority_2_handle.set_linewidths(3 * d[1])
            self.plt_priority_2_handle.set_alpha(p[1])

            self.plt_priority_3B_handle.set_linewidths(1+3 * d[2])
            self.plt_priority_3_handle.set_linewidths(3 * d[2])
            self.plt_priority_3_handle.set_alpha(p[2])

            self.plt_priority_4B_handle.set_linewidths(1+3 * d[3])
            self.plt_priority_4_handle.set_linewidths(3 * d[3])
            self.plt_priority_4_handle.set_alpha(p[3])

            self.plt_priority_5B_handle.set_linewidths(1+3 * d[4])
            self.plt_priority_5_handle.set_linewidths(3 * d[4])
            self.plt_priority_5_handle.set_alpha(p[4])

            self.plt_priority_6B_handle.set_linewidths(1+3 * d[5])
            self.plt_priority_6_handle.set_linewidths(3 * d[5])
            self.plt_priority_6_handle.set_alpha(p[5])

            self.plt_priority_7B_handle.set_linewidths(1+3 * d[6])
            self.plt_priority_7_handle.set_linewidths(3 * d[6])
            self.plt_priority_7_handle.set_alpha(p[6])

            self.plt_priority_8B_handle.set_linewidths(1+3 * d[7])
            self.plt_priority_8_handle.set_linewidths(3 * d[7])
            self.plt_priority_8_handle.set_alpha(p[7])

            # Updating Biological Clock time.
            #print(self.miro.platform_state.rtc_hrs)
            ang = np.deg2rad(270-(self.miro.platform_state.rtc_hrs*360.0/24.0))
            self.ax_bioclock_handle.set_positions((0.0, 0.0), (np.cos(ang) * 0.7, np.sin(ang) * 0.7))

    # ==========================

    def update_SalienceVisualZoom(self, i):
        if rospy.core.is_shutdown():
            return

        caml = self.miro.caml_fifo.latest()
        camr = self.miro.camr_fifo.latest()
        if (caml is not None) and (camr is not None):
            self.plt_camera_l_handle_SAM.set_data(caml)
            self.plt_camera_r_handle_SAM.set_data(camr)

        priw = self.miro.priw_fifo.latest()
        if (priw is not None):
            self.plt_priw_handle_SAM.set_data(priw[:, :, 0])

        pril = self.miro.pril_fifo.latest()
        prir = self.miro.prir_fifo.latest()
        if (pril is not None) and (prir is not None):
            self.plt_PRI_l_handle_SAM.set_data(pril)
            self.plt_PRI_r_handle_SAM.set_data(prir)

    # =========================

    def update_BasalGangliaZoom(self, i):
        if rospy.core.is_shutdown():
            return

        if self.miro.core_state is not None:
            for bar, h in zip(self.plt_GPRWin_handle, self.miro.core_state.selection):
                bar.set_height(h)

    # =========================

    def update_BasalGanglia_InOutZoom(self, i):
        if rospy.core.is_shutdown():
            return

        if self.miro.core_state is not None:
            for bar, h in zip(self.plt_priority_handle, self.miro.core_state.priority):
                bar.set_height(h)

            for bar, h in zip(self.plt_disinhibition_handle, self.miro.core_state.disinhibition):
                bar.set_height(h)

    # =========================

    def update_AffectStateWindow(self, i):
        if rospy.core.is_shutdown():
            return

        if self.miro.core_state is not None:
            self.plt_circle_red_handle_AS.set_offsets([self.miro.core_state.emotion.valence * 16.0 - 8.0, self.miro.core_state.emotion.arousal * 16.0 - 8.0])
            self.plt_circle_blue_handle_AS.set_offsets([self.miro.core_state.mood.valence * 16.0 - 8.0, self.miro.core_state.mood.arousal * 16.0 - 8.0])
            self.plt_circle_yellow_handle_AS.set_offsets([self.miro.core_state.sleep.wakefulness * 16.0 - 8.0, self.miro.core_state.sleep.pressure * 16.0 - 8.0])

    # =========================
    # Events callback functions.
    def callback_WinClose(self, event):
        print('Figure closed!')
        global animate_MainWindow
        animate_MainWindow = True
        self.show_pri = 1

################################################################
if __name__ == "__main__":
    # Setting the back/static image(s).
    MainGI = MiroGI()

################################################################
