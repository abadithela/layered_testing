import numpy as np
import os
import sys
sys.path.append("../carlo")
from world import World
from agents import Car, RectangleBuilding, Pedestrian, Painting
from geometry import Point
import time
import pdb
human_controller = False

dt = 0.1 # time steps in terms of seconds. In other words, 1/dt is the FPS.
w = World(dt, width = 120, height = 120, ppm = 6) # The world is 120 meters by 120 meters. ppm is the pixels per meter.

# Let's add some sidewalks and RectangleBuildings.
# A Painting object is a rectangle that the vehicles cannot collide with. So we use them for the sidewalks.
# A RectangleBuilding object is also static -- it does not move. But as opposed to Painting, it can be collided with.
# For both of these objects, we give the center point and the size.

# w.add(RectangleBuilding(Point(72.5, 107.5), Point(95, 25))) #
# w.add(RectangleBuilding(Point(7.5, 107.5), Point(15, 25)))
#
# w.add(RectangleBuilding(Point(7.5, 40), Point(15, 80)))
#
# w.add(RectangleBuilding(Point(72.5, 40), Point(95, 80)))

# w.add(RectangleBuilding(Point(72.5, 107.5), Point(95, 25)))

w.add(RectangleBuilding(Point(7.5, 107.5), Point(225, 25)))

w.add(RectangleBuilding(Point(7.5, 35), Point(225, 70)))
#
# w.add(RectangleBuilding(Point(72.5, 40), Point(95, 80)))


# A Car object is a dynamic object -- it can move. We construct it using its center location and heading angle.
# system under test
c1 = Car(Point(10,90), 0.0)
w.add(c1)
c1.velocity = Point(3.0,0) # We can also specify an initial velocity just like this.

c2 = Car(Point(10,80), 0.0, 'blue')
c2.velocity = Point(3.0,0) # We can also specify an initial velocity just like this.
w.add(c2)

c3 = Car(Point(25,80), 0.0, 'blue')
c3.velocity = Point(3.0,0) # We can also specify an initial velocity just like this.
w.add(c3)


w.render() # This visualizes the world we just constructed.
pdb.set_trace()
print("Done")

# if not human_controller:
#     # Let's implement some simple scenario with all agents
#     c1.set_control(0, 0.35)
#     c2.set_control(0, 0.05)
#     c3.set_control(0, 0.05)
#     for k in range(400):
#         # All movable objects will keep their control the same as long as we don't change it.
#         if k == 100: # Let's say the first Car will release throttle (and start slowing down due to friction)
#             c1.set_control(0, 0)
#         elif k == 200: # The first Car starts pushing the brake a little bit. The second Car starts turning right with some throttle.
#             c1.set_control(0, -0.02)
#         elif k == 325:
#             c1.set_control(0, 0.8)
#             c2.set_control(-0.45, 0.3)
#         elif k == 367: # The second Car stops turning.
#             c2.set_control(0, 0.1)
#         w.tick() # This ticks the world for one time step (dt second)
#         w.render()
#         time.sleep(dt/4) # Let's watch it 4x
#
#         if w.collision_exists(): # Or we can check if there is any collision at all.
#             print('Collision exists somewhere...')
#     w.close()
#
# else: # Let's use the steering wheel (Logitech G29) for the human control of car c1
#     c2.set_control(0, 0.35)
#
#     from interactive_controllers import SteeringWheelController
#     controller = SteeringWheelController(w)
#     for k in range(400):
#         c1.set_control(controller.steering, controller.throttle)
#         w.tick() # This ticks the world for one time step (dt second)
#         w.render()
#         time.sleep(dt/4) # Let's watch it 4x
#         if w.collision_exists():
#             import sys
#             sys.exit(0)
