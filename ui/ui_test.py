#!/usr/bin/env python
################################################################
import rospy
import interface_lib as lib
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ==============================================================

class MiroGI():
    def __init__(self):
        rospy.init_node("miro_ros_client_py", anonymous=True)
        self.miro = lib.miro_ros_client()

        Back_image = plt.imread('../documents/biomimetic_core.png')
        self.emoji_1 = plt.imread('../documents/happy.png')

        # Initializing moving images.
        self.screen_size = [800, 450]
        self.fig, self.axes = plt.subplots(nrows=1, ncols=1, figsize=(9, 5))
        plt.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.0, wspace=0.0, hspace=0.0)
        self.pltManager = plt.get_current_fig_manager()
        self.pltManager.resize(*self.pltManager.window.maxsize())

        # Setting the back/static image(s).
        self.axes.imshow(Back_image, zorder=0, extent=[0, self.screen_size[0], 0, self.screen_size[1]])
        self.axes.axis('off')  # clear x- and y-axes
        self.axes.set_xlim([0, self.screen_size[0]])
        self.axes.set_ylim([0, self.screen_size[1]])

        # Initializing moving images.
        self.emoji_handle = self.axes.imshow(self.emoji_1, zorder=1, extent=[0, 30, 0, 30])

        # cursor = Cursor()
        cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)

        ani = animation.FuncAnimation(self.fig, self.plot_updatefig, interval=500)
        plt.show()

    def img_plot(self, image, size, pos, zorder, ofset_centre=True):
        if ofset_centre:
            extent = [pos[0] - (size[0] / 2), pos[0] + (size[0] / 2), \
                      pos[1] - (size[1] / 2), pos[1] + (size[1] / 2) ]
        else:
            extent = [pos[0], pos[0] + size[0], pos[1], pos[1] + size[1]]

        img_handle = self.axes.imshow(image, zorder=zorder, extent=extent)
        return img_handle

    def plot_updatefig(self, i):
        if rospy.core.is_shutdown():
            return
        while self.miro.platform_sensors is None:
            pass
        self.miro.update_data()

        # Plotting animation
        self.emoji_handle.remove()
        x = self.miro.accel_head.x
        y = self.miro.accel_head.y
        self.emoji_handle = self.img_plot(self.emoji_1, [10, 10], [(x*100)+354, (y*100)+101], 1)

        self.caml = self.img_plot(self.miro.image_caml, [32, 24], [400, 200], 1)

    # Getting the cursor click position.
    def onclick(self, event):
        print('button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
              (event.button, event.x, event.y, event.xdata, event.ydata))

################################################################
if __name__ == "__main__":
    # Setting the back/static image(s).
    MainGI = MiroGI()

################################################################
