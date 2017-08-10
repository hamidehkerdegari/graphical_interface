#!/usr/bin/env python
################################################################
import rospy
import interface_lib as lib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# ==============================================================

class MiroGI():
    def __init__(self):
        rospy.init_node("miro_ros_client_py", anonymous=True)
        self.miro = lib.miro_ros_client()

        img_back = plt.imread('../documents/biomimetic_core.png')
        self.img_emoji1 = plt.imread('../documents/happy.png')


        # Initializing the main window.
        self.screen_size = [800, 450]
        self.fig_main, self.ax_main = plt.subplots(nrows=1, ncols=1, figsize=(9, 5))
        self.fig_main.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.0, wspace=0.0, hspace=0.0)
        self.pltManager = plt.get_current_fig_manager()
        self.pltManager.resize(*self.pltManager.window.maxsize()) # Make full screen

        # Setting the back/static image(s).
        self.ax_main.imshow(img_back, zorder=0, extent=[0, self.screen_size[0], 0, self.screen_size[1]])
        self.ax_main.axis('off')  # clear x- and y-axes
        self.ax_main.set_xlim([0, self.screen_size[0]])
        self.ax_main.set_ylim([0, self.screen_size[1]])


        # Initializing the GPR plot.
        self.ax_GPR = lib.add_subplot(self.ax_main, self.fig_main, [0.2, 0.2, 0.1, 0.1])
        self.index = np.arange(8)
        self.bar_width = 0.9
        self.opacity = 0.4
        #for spine in plt.gca().spines.values():  # Get rid of the frame
        #    spine.set_visible(False)
        self.ax_GPR.patch.set_visible(False)  # Remove backgrounf
        self.ax_GPR.tick_params(top='off', bottom='off', left='off', right='off', labelleft='off', labelbottom='on')
        self.ax_GPR.set_xticks(self.index + self.bar_width / 2)
        self.ax_GPR.set_xticklabels(['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'])
        self.plt_GPR_handle = self.ax_GPR.bar(self.index, (20, 35, 30, 35, 27, 15, 35, 20), self.bar_width, zorder=1, alpha=self.opacity, color='b')


        #  Initializing moving circle.
        self.ax_circle_red = lib.add_subplot(self.ax_main, self.fig_main, [0.2, 0.2, 0.3, 0.3])
        #for spine in plt.gca().spines.values():  # Get rid of the frame
        #    spine.set_visible(False)
        self.ax_circle_red.patch.set_visible(False)  # Remove backgrounf
        self.ax_circle_red.tick_params(top='off', bottom='off', left='off', right='off', labelleft='off', labelbottom='off')
        self.ax_circle_red.set_xlim([-10, 10])
        self.ax_circle_red.set_ylim([-10, 10])
        self.ax_circle_red.set_aspect('equal')
        self.plt_emoji_handle = self.ax_circle_red.scatter(2, 7, s=100, c='r', alpha=0.9)


        # cursor = Cursor()
        cid = self.fig_main.canvas.mpl_connect('button_press_event', self.onclick)

        ani = animation.FuncAnimation(self.fig_main, self.plot_updatefig, interval=500)
        plt.show()

    def img_plot(self, ax, image, size, pos, zorder, ofset_centre=True):

        if ofset_centre:
            extent = [pos[0] - (size[0] / 2), pos[0] + (size[0] / 2), \
                      pos[1] - (size[1] / 2), pos[1] + (size[1] / 2) ]
        else:
            extent = [pos[0], pos[0] + size[0], pos[1], pos[1] + size[1]]

        img_handle = ax.imshow(image, zorder=zorder, extent=extent)
        return img_handle

    def plot_updatefig(self, i):
        if rospy.core.is_shutdown():
            return
        while (self.miro.platform_sensors is None) or (self.miro.core_state is None):
            pass
        self.miro.update_data()

        # Plotting animation
        self.plt_emoji_handle.remove()
        x = self.miro.accel_head.x
        y = self.miro.accel_head.y
        self.plt_emoji_handle = self.ax_circle_red.scatter(x, y, s=100, c='r', alpha=0.9)


        self.plt_GPR_handle.remove()
        self.plt_GPR_handle = self.ax_GPR.bar(self.index, self.miro.selection, self.bar_width, zorder=1, alpha=self.opacity, color='b')
        print self.miro.selection

    # Getting the cursor click position.
    def onclick(self, event):
        print('button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
              (event.button, event.x, event.y, event.xdata, event.ydata))

################################################################
if __name__ == "__main__":
    # Setting the back/static image(s).
    MainGI = MiroGI()

################################################################
