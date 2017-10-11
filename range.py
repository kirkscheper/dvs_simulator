#!/usr/bin/env python

""" 
Author: F. Paredes Valles
Created: 27-06-2017
"""

import os
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

matplotlib.rcParams.update({'font.size': 12})

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


f = plt.figure(1, figsize=(12, 5))
ax = plt.subplot(121, projection='polar')

ax.plot(th, vx, label=r'|$\omega_x$| [1/s]')
ax.plot(th, vy, label=r'|$\omega_y$| [1/s]')
ax.plot(th, vx2, label=r'|$\omega_x$| [1/s]', color='#1f77b4')
ax.plot(th, vy2, label=r'|$\omega_y$| [1/s]', color='#ff7f0e')
ax.set_title("Set 1: Training", va='bottom', fontsize=20)

plt.fill_between(th, vx, vx2, alpha=0.2)
plt.fill_between(th, vy, vy2, alpha=0.2)

for i in xrange(0,len(th)):
	ax.plot([th[i], th[i]], [vx[i], vx2[i]], color='#1f77b4', alpha=0.2, linestyle='--')
	ax.plot([th[i], th[i]], [vy[i], vy2[i]], color='#ff7f0e', alpha=0.2, linestyle='--')

#########################################################################################

# compute the angles
thDelta = 33.5
thEnd   = (360-thDelta)*np.pi/180.0
th      = np.linspace(0, thEnd, num=360/thDelta)
th = np.append(th, 0)

# velocity vector
velocities = [1.62]

vx = []
vy = []
for i in xrange(0,len(th)):
	vx.append(abs(velocities[0]*np.cos(th[i]))*2)
	vy.append(abs(velocities[0]*np.sin(th[i]))*2)

velocities = [0.375]
vx2 = []
vy2 = []
for i in xrange(0,len(th)):
	vx2.append(abs(velocities[0]*np.cos(th[i]))*2)
	vy2.append(abs(velocities[0]*np.sin(th[i]))*2)

ax2 = plt.subplot(122, projection='polar')
ax2.plot(th, vx, label=r'|$\omega_x$| [1/s]')
ax2.plot(th, vy, label=r'|$\omega_y$| [1/s]')
ax2.plot(th, vx2, color='#1f77b4')
ax2.plot(th, vy2, color='#ff7f0e')
ax2.set_title("Set 2: Validation", va='bottom', fontsize=20)

plt.fill_between(th, vx, vx2, alpha=0.2)
plt.fill_between(th, vy, vy2, alpha=0.2)

for i in xrange(0,len(th)):
	ax2.plot([th[i], th[i]], [vx[i], vx2[i]], color='#1f77b4', alpha=0.2, linestyle='--')
	ax2.plot([th[i], th[i]], [vy[i], vy2[i]], color='#ff7f0e', alpha=0.2, linestyle='--')

leg = plt.legend(fontsize=18)

plt.draw() # Draw the figure so you can find the positon of the legend. 
bb = leg.get_bbox_to_anchor().inverse_transformed(ax.transAxes)
xOffset = -0.625
bb.x0 += xOffset
bb.x1 += xOffset
leg.set_bbox_to_anchor(bb, transform = ax.transAxes)
plt.tight_layout()

f.set_rasterized(True)
#plt.savefig('/home/fedepare/Preliminary Thesis Report/Include/images/polar.eps', dpi=255)

plt.show()