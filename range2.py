#!/usr/bin/env python

""" 
Author: F. Paredes Valles
Created: 27-06-2017
"""

import os
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from math import fabs

matplotlib.rcParams.update({'font.size': 14})

# compute the angles
thDelta = 15
thEnd   = (360-thDelta)*np.pi/180.0
th      = np.linspace(0, thEnd, num=360/15)
th = np.append(th, 0)

# velocity vector
velocities = [1.5]

vx = []
vy = []
maxx = []
for i in xrange(0,len(th)):
	vx.append(abs(velocities[0]*np.cos(th[i]))*2)
	vy.append(abs(velocities[0]*np.sin(th[i]))*2)
	maxx.append(max(abs(velocities[0]*np.cos(th[i]))*2, abs(velocities[0]*np.sin(th[i]))*2))

velocities = [0.25]
vx2 = []
vy2 = []
minn = []
for i in xrange(0,len(th)):
	vx2.append(abs(velocities[0]*np.cos(th[i]))*2)
	vy2.append(abs(velocities[0]*np.sin(th[i]))*2)
	minn.append(min(abs(velocities[0]*np.cos(th[i]))*2, abs(velocities[0]*np.sin(th[i]))*2))

f = plt.figure(1, figsize=(8, 5))
ax = plt.subplot(111, projection='polar')

ax.plot(th, vx, color='#7f7f7f')
ax.plot(th, vy, color='#7f7f7f')
ax.plot(th, vx2, color='#7f7f7f')
ax.plot(th, vy2, color='#7f7f7f')
ax.set_title(r'Set 3: $r = 1.50$ m', va='bottom', fontsize=16)

plt.fill_between(th, vx, vx2, alpha=0.2, color='#7f7f7f')
plt.fill_between(th, vy, vy2, alpha=0.2, color='#7f7f7f')

for i in xrange(0,len(th)):
	ax.plot([th[i], th[i]], [vx[i], vx2[i]], color='#7f7f7f', alpha=0.2, linestyle='--')
	ax.plot([th[i], th[i]], [vy[i], vy2[i]], color='#7f7f7f', alpha=0.2, linestyle='--')

#########################################################################################

venFlow150 = np.loadtxt('circle_050.txt')
venFlow150 = venFlow150[:-1,:]

th = []
vx = []
vy = []
for i in xrange(0,len(venFlow150)):
	angle = np.arctan(venFlow150[i,2]/venFlow150[i,1])
	if venFlow150[i,1] >= 0:
		pass
	else:
		angle += np.pi

	th.append(angle)
	vx.append(fabs(venFlow150[i,1]))
	vy.append(fabs(venFlow150[i,2]))

ax.plot(th, vx, label=r'|$\omega_x$| [1/s]')
ax.plot(th, vy, label=r'|$\omega_y$| [1/s]')

leg = plt.legend(fontsize=16, frameon=False)

for line in leg.get_lines():
    line.set_linewidth(2)

plt.draw() # Draw the figure so you can find the positon of the legend. 
bb = leg.get_bbox_to_anchor().inverse_transformed(ax.transAxes)
xOffset = 0.55
bb.x0 += xOffset
bb.x1 += xOffset
leg.set_bbox_to_anchor(bb, transform = ax.transAxes)
plt.tight_layout()

f.set_rasterized(True)
#plt.savefig('/home/fedepare/Preliminary Thesis Report/Include/images/polar_150.eps', dpi=255)
plt.show()