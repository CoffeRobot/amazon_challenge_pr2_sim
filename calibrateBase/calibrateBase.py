#! /usr/bin/python

import rospy
import math


from baseMove import *
from baseScan import *
import matplotlib.pyplot as plt

rospy.init_node('calibrate_base_node', anonymous=True)


# rospy.sleep(10)
# rate = rospy.Rate(10	.0)
# 

position = [0,0]
angle = 0

# bm = baseMove(verbose=False)

# bm.setPosTolerance(0.001)
# bm.setAngTolerance(0.001)
# bm.setLinearGain(10)
# bm.setAngularGain(10)


# bm.go(position, angle)

# bm.goAngle(angle)
# bm.goPosition(destination)
# bm.goAngle(angle)




bs = baseScan()

pc = bs.getCloud()

legs = bs.findLegs()
shelfOri, shelfRot = bs.getShelfFrame()

print shelfRot


x = []
y= []

for i in range(len(pc)):
	x.append(pc[i][0])
	y.append(pc[i][1])

plt.plot(y, x, 'wo')
plt.plot([0], [0], 'g*')

plt.plot(legs[0][1], legs[0][0], 'ro')
plt.plot(legs[1][1], legs[1][0], 'ro')

plt.plot(shelfOri[1], shelfOri[0], 'ro')
l = 0.4
plt.plot([shelfOri[1], shelfOri[1] + l*sin(shelfRot)], [shelfOri[0], shelfOri[0] + l*cos(shelfRot)], 'r')
plt.plot([shelfOri[1], shelfOri[1] + l*cos(shelfRot)], [shelfOri[0], shelfOri[0] - l*sin(shelfRot)], 'g')

plt.axis([-2, 2, -1, 3])
plt.gca().invert_xaxis()
plt.show()

