#!/usr/local/bin/python
#
#
#
#
import sys
sys.path.append('..') # enable importing modules from an upper directory:
import os
from PIL import Image
import matplotlib as mpl
mpl.use('tkagg')
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import time
from ipdb import set_trace as st

show_axes = False
dir_path = os.path.dirname(os.path.realpath(__file__))
road_fig = os.path.dirname(dir_path) + '/layered_testing/imglib/test_road.png'


# creates figure
fig = plt.figure()
ax = fig.add_axes([0,0,1,1]) # get rid of white border
if not show_axes:
    plt.axis('off')

# sampling time
dt = 1
duration = 10

def get_background():
    return Image.open(road_fig)

background = get_background()

def animate(frame_idx):
    # st()
    global background
    ax.clear()
    t0 = time.time()
    # update cars
    # draw cars
    # update background
    the_road = [ax.imshow(background, origin="lower")] # update the stage
    background.close()
    background = get_background()

    all_artists = the_road # + the_cars
    t1 = time.time()
    elapsed_time = (t1 - t0)

    return all_artists

try:
    t0 = time.time()
    # st()
    animate(0)
    t1 = time.time()
    interval = (t1 - t0)
    ani = animation.FuncAnimation(fig, animate, frames=int(duration/dt), interval=interval, blit=True, repeat=False) # by default the animation function loops so set repeat to False in order to limit the number of frames generated to num_frames
    # if options.save_video:
    #     #Writer = animation.writers['ffmpeg']
    #     writer = animation.FFMpegWriter(fps = options.speed_up_factor*int(1/dt), metadata=dict(artist='Traffic Intersection Simulator'), bitrate=-1)
    #     now = str(datetime.datetime.now())
    #     ani.save('../movies/' + now + '.avi', dpi=200, writer=writer)
    plt.show()
    t2 = time.time()
    print('Total elapsed time: ' + str(t2-t0))
except:
    st()
