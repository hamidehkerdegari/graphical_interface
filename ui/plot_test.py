import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

image = plt.imread('/home/saeid/Desktop/4gwqctkhxr8lxboz.jpg')
image2 = plt.imread('/home/saeid/Dropbox/Image.png')

fig, axes = plt.subplots(nrows=1, ncols=1, figsize=(9, 5))
plt.subplots_adjust(left=0.08, right=0.98, top=0.97, bottom=0.11, wspace=0.06, hspace=0.03)

axes.imshow(image, zorder=0, extent=[0, 1024, 0, 768])
axes.axis('off')  # clear x- and y-axes


emoji_handle = axes.imshow(image2, zorder=1)
dot_handle = axes.scatter(0, 0, zorder=2)
def updatefig(i):
    global emoji_handle, dot_handle
    x = np.random.random()
    y = np.random.random()
    a.remove()
    b.remove()
    a = axes.imshow(image2, zorder=1, extent=[x * 100, x * 100 + 100, y * 100, y * 100 + 100])
    b = axes.scatter(x * 1024, y * 768, zorder=2)

ani = animation.FuncAnimation(fig, updatefig, interval=100)
plt.show()

