#!/usr/bin/env python
################################################################
import rospy
import interface_lib as lib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
# ==============================================================


def plot_updatefig(i):
    if rospy.core.is_shutdown():
        return
    while miro.platform_sensors is None:
        pass
    miro.update_data()

    # Plotting animation
    global emoji_handle
    emoji_handle.remove()
    x = miro.accel_head.x
    y = miro.accel_head.y
    emoji_handle = axes.imshow(emoji_1, zorder=1, extent=[x*100+1024/2, x*100+30+1024/2, y*100+768/2, y*100+30+768/2])
# ==============================================================


class Cursor():
    # Getting the cursor click position.
    def onclick(self, event):
        print('button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
              (event.button, event.x, event.y, event.xdata, event.ydata))

################################################################
if __name__ == "__main__":
    rospy.init_node("miro_ros_client_py", anonymous=True)
    miro = lib.miro_ros_client()

    Back_image = plt.imread('../documents/biomimetic_core.png')
    emoji_1 = plt.imread('../documents/happy.png')

    fig, axes = plt.subplots(nrows=1, ncols=1, figsize=(9, 5))
    plt.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.0, wspace=0.0, hspace=0.0)
    manager = plt.get_current_fig_manager()
    manager.resize(*manager.window.maxsize())
    cursor = Cursor()
    cid = fig.canvas.mpl_connect('button_press_event', cursor.onclick)

    # Setting the back/static image(s).
    axes.imshow(Back_image, zorder=0, extent=[0, 1024, 0, 768])
    axes.axis('off')  # clear x- and y-axes
    axes.set_xlim([0, 1024])
    axes.set_ylim([0, 768])

    # Initializing moving images.
    emoji_handle = axes.imshow(emoji_1, zorder=1, extent=[0, 30, 0, 30])

    ani = animation.FuncAnimation(fig, plot_updatefig, interval=100)
    plt.show()

################################################################
