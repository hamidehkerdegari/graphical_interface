#!/usr/bin/env python
################################################################
import rospy
from interface_lib import *

#=======================================
def updatefig(i):
    if rospy.core.is_shutdown():
        return
    while miro.platform_sensors is None:
        pass
    miro.update_data()
    print miro.accel_head.x
    print miro.accel_head.y


    # ==============================
    global a, b
    x = miro.accel_head.x
    y = miro.accel_head.y
    a.remove()
    b.remove()
    a = axes.imshow(image2, zorder=1, extent=[x * 100, x * 100 + 30, y * 100, y * 100 + 30])
    #b = axes.scatter(x * 1024, y * 768, zorder=2)
    b = axes.scatter(x * 100, y * 100, zorder=2)

#=======================================

if __name__ == "__main__":
    rospy.init_node("miro_ros_client_py", anonymous=True)
    miro = miro_ros_client()
    #miro.loop()

    #================================
    import numpy as np
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation

    image = plt.imread('../documents/biomimetic_core.png')
    image2 = plt.imread('../documents/happy.png')

    fig, axes = plt.subplots(nrows=1, ncols=1)
    plt.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.0, wspace=0.06, hspace=0.03)

    axes.imshow(image, zorder=0, extent=[0, 1024, 0, 768])
    axes.axis('off')  # clear x- and y-axes

    a = axes.imshow(image2, zorder=1, extent=[0, 30, 0, 30])
    b = axes.scatter(0, 0, zorder=2)

    manager = plt.get_current_fig_manager()
    manager.resize(*manager.window.maxsize())

    ani = animation.FuncAnimation(fig, updatefig, interval=100)
    plt.show()



