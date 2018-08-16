#!/usr/bin/python3.6

import csv
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D # 3D plot
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# draw a vector
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

import time

from class_obstacle import * # Custom obstacle class

# Define function
class Arrow3D(FancyArrowPatch):

    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        FancyArrowPatch.draw(self, renderer)

# ------ Start script ------------
print('')
print('Start script')
print('')

# Set true for first run to reevaluate data
newImport = False
if newImport:
    bagName = '2018-07-05-12-26-17' # first video simulation
    # fileName = '2018-06-28-22-09-33'
    # bagName = '2018-06-28-22-09-33partial'
    # fileName='2018-07-05-12-26-17'
    # fileName = '2018-08-15-23-19-25partial'
    # fileName = '2018-08-15-23-38-28partial'
    # bagName = '2018-08-15-23-39-47partial'
    # bagName = '2018-08-16-00-47-25partial'
    # bagName = '2018-08-16-01-15-41partial'
    # bagName = '2018-08-16-02-29-54partial'

    csv = np.genfromtxt('MeasurementClean/' + bagName + '.csv', delimiter=";")

    # import pdb; pdb.set_trace() # Break point python

    t = csv[1:, 0]
    t = t - t[0]

    print('Number of measurements', len(t))
    pos = csv[1:, 1:]

    # Cut corresponding to t_range
    # t_range=[t[0], t[-1]]
    t_range = [40, 100] # Regular movement
    # t_range = [40, 100] # Regular movement
    n_range = [np.where(t >= t_range[0])[0][0], np.where(t >= t_range[1])[0][0]]

    t = t[n_range[0]: n_range[1]+1]

    # Correct time // somehow there are certian wrong timestamps...
    # e.g. moving averagefilter
    N=30

    t_temp = t[N:-N]
    for ii in range(1,N):
        t_temp = t_temp + t[N+ii:-N+ii] + t[N-ii:-N-ii]

    t = t_temp/(2*N-1)
    t = t-t[0]

    # adjust all values
    n_range[0] = n_range[0]+N
    n_range[1] = n_range[1]-N

    # dt = t[1:]-t[0:-1]
    # wrongTime_pos = np.where(dt<0)[0]


    pos = pos[n_range[0]: n_range[1]+1, :]

    N_link = 7
    posEE = np.zeros((pos.shape[0], 3)) # Sum up all links -- end effector position

    for ii in range(3):
        for jj in range(N_link):
            posEE[:,ii] = posEE[:,ii] + pos[:,ii+jj*3]

fig = plt.figure()
ax = []

ii=0
ax.append(fig.add_subplot(3,1,ii+1))
# ax[ii].plot(t,posEE[:,ii], 'r', marker='.', linestyle='')
ax[ii].plot(t,posEE[:,ii], 'r')
ax[ii].set_xlim(t[0], t[-1])
ax[ii].set_ylim(-0.9, 0.9)
ax[ii].set_xlim(t[0], t[-1])
ax[ii].tick_params(labelbottom=False)
ax[ii].grid(True)
ax[ii].set_ylabel(r'$\xi_{}$ [m]'.format(ii+1))

ii=1
ax.append(fig.add_subplot(3,1,ii+1))
ax[ii].plot(t,posEE[:,ii], 'g')
ax[ii].set_xlim(t[0], t[-1])
ax[ii].tick_params(labelbottom=False)
ax[ii].set_ylim(-0.9,0.9)
ax[ii].grid(True)
ax[ii].set_ylabel(r'$\xi_{}$ [m]'.format(ii+1))

ii=2
ax.append(fig.add_subplot(3,1,ii+1))
ax[ii].plot(t,posEE[:,ii],'b')
ax[ii].set_xlim(t[0], t[-1])
ax[ii].set_ylim(-0.1,1.1)
ax[ii].grid(True)
ax[ii].set_ylabel(r'$\xi_{}$ [m]'.format(ii+1))

ax[ii].set_xlabel('Time [s]')

nPlot = 0

fig3 = plt.figure(figsize=plt.figaspect(0.44)*1.0)
# fig3 = plt.figure()
ax3 = fig3.add_subplot(111, projection='3d')

if obstaclesShow:
    ### Create obstacles
    obs = []

    # Virutal -- conveyer belt
    a=[1.0,.09,.15]
    x0_hat=[-0,0.5,0.15] # In 
    p=[10.,1.,2.]
    sf = 1.
    
    th_r=[0,0,0]
    th_r=[th_r[i]/180.*pi for i in range(3)]
    
    obs.append(Obstacle(a=a, p=p, x0=x0_hat, th_r=th_r, sf=sf))

    ##### Basket -- Wall
    a=[0.70,.11,.25]
    x0_hat=[-.28,-0.275-0.03,0.25] # In 
    p=[10.,1.,2.]
    sf = 1
    th_r=[0,0,0]
    
    th_r=[th_r[i]/180.*pi for i in range(3)]
    
    obs.append(Obstacle(a=a, p=p, x0=x0_hat,th_r=th_r, sf=sf))

    ##### Basket -- Floor
    a=[0.70,.11,.25]
    x0_hat=[-.28,-0.275-0.22,0.0] # In 
    p=[10.,1.,2.]
    sf = 1

    th_r=[90,0,0]
    th_r=[th_r[i]/180.*pi for i in range(3)]
    
    obs.append(Obstacle(a=a, p=p, x0=x0_hat,th_r=th_r, sf=sf))

    cols = [[0.2,0.3,0.2],
            [.8,0.4,0],
            [.8,0.4,0]]
    
    for oo in range(len(obs)):
    # for oo in [1]:
        obs[oo].draw_ellipsoid(20)
        obsGrid = np.array((obs[oo].x_obs_sf))

        obsVarShape=(10,20)
        ax3.plot_wireframe(obsGrid[:,0].reshape(obsVarShape),
                           obsGrid[:,1].reshape(obsVarShape),
                           obsGrid[:,2].reshape(obsVarShape), color=[0.2,0.2,0.2])

        
        # ax3.plot_surface(obsGrid[:,0].reshape(obsVarShape),
                         # obsGrid[:,1].reshape(obsVarShape),
                         # obsGrid[:,2].reshape(obsVarShape),
                         # color=cols[oo], alpha=0.5, rstride=4, cstride=4)

# obs_polygon.append(
    # ax3.plot_surface(
        # np.reshape([obs[n].x_obs[i][0] for i in range(len(obs[n].x_obs))],
                   # (N_resol,-1)),
        # np.reshape([obs[n].x_obs[i][1] for i in range(len(obs[n].x_obs))],
                   # (N_resol,-1)),
        # np.reshape([obs[n].x_obs[i][2] for i in range(len(obs[n].x_obs))],
                   # (N_resol, -1))  )  )


ax3.plot(posEE[:,0], posEE[:,1], posEE[:,2], '-', color=[0,0,0.6], label='Trajectory')
arrColor = [0.5, 0, 0]
ax3.plot([],[],[], '-', color=arrColor, label='Iniital DS')

xRange = [-0.9,0.9]
yRange = [-0.9,0.9]
zRange = [-0.01, 0.7]
# Pickup, drop off and via location
attrPos = np.array([[-0.44, 0.61, 0.54],
                    [-0.44, 0.61, 0.34],
                    [-0.272, -0.420, 0.300]])

for ii in range(3):
    arrow = Arrow3D([attrPos[ii, 0], attrPos[(ii+1)%3, 0]],
                    [attrPos[ii, 1], attrPos[(ii+1)%3, 1]],
                    [attrPos[ii, 2], attrPos[(ii+1)%3, 2]],
                    mutation_scale=20, lw=2, arrowstyle="-|>", color=arrColor, linestyle='-', label='Initial DS')

    ax3.add_artist(arrow)
    # ax3.plot([attrPos[ii, 0], attrPos[(ii+1)%3, 0]],
             # [attrPos[ii, 1], attrPos[(ii+1)%3, 1]],
             # [attrPos[ii, 2], attrPos[(ii+1)%3, 2]],
              # 'k', linestyle='--') # Via point

ax3.plot([-0.44], [0.61], [0.34], c='k', marker='s', linestyle='', markersize=10, label='Via point') # Via point
ax3.plot([-0.44], [0.61], [0.54], c='k', marker='v', linestyle='', markersize=10, label='Pick up') # Pickup 
ax3.plot([-0.272], [-0.420], [0.300], c='k', marker='*', linestyle='', markersize=10, label='Drop off') # Drop off

# xx = xRange + [xRange[1], xRange[0]]
# yy = [yRange[0]] + yRange + [yRange[1]]
# zz = 4 * [zRange[0]]
# verts = [list(zip(xx, yy, zz))]
# ax3.add_collection3d(Poly3DCollection(verts, alpha=0.4, facecolors='gray'), zs='z')

# Draw static obstacles
obstaclesShow = True
# Define obstacles

# Draw floor
# ax3.set_xticks([])
# ax3.set_xticklabels([])
# ax3.tick_params(labelbottom=False, labelright=False, labelleft=False)
ax3.set_xlabel(r'$\xi_1$')
ax3.set_ylabel(r'$\xi_2$')
ax3.set_zlabel(r'$\xi_3$')
ax3.set_xlim(xRange[0], xRange[1])
ax3.set_ylim(yRange[0], yRange[1])
# ax3.set_zlim(zRange[0], zRange[1])
# ax.pbaspect = [0.6, 0.6, 0.2]
# ax3.set_aspect('equal')

# Define postition of obstacles / attractor
ax3.legend()
plt.show()
# import pdb; pdb.set_trace() # Break point python
            

robotPlot = False
if robotPlot:
    fig3 = plt.figure(figsize=plt.figaspect(0.5)*1.0)
    # plt.figure()
    ax3 = fig3.add_subplot(111, projection='3d')

    plt.ion() # show during script
    # dt=0.1
    # for nPlot in range(t.shape[0]): # Dynamic loop iteration
    for nPlot in [10]: # looping does not work, try Animation library
        dPos = [[0,0] for i in range(3)]
        for ii in range(N_link):
            print('ii',ii)
            for jj in range(3):
                dPos[jj] = ([dPos[jj][1], dPos[jj][1]+pos[nPlot, ii*3+jj]])
            ax3.plot(dPos[0], dPos[1], dPos[2]) #, linewidth='20', color=[1,0.55,0], solid_capstyle='round')
            print(dPos)

            # if ii > 0:
                # ax3.plot([dPos[0][0]], [dPos[1][0]], [dPos[2][0]], linewidth='0', color='k', marker='.', markersize=10)
            ax3.plot(dPos[0], dPos[1], dPos[2], linewidth='0', color='k', marker='.', markersize=10)

            # plt.show()
            # import pdb; pdb.set_trace() # Break point python

            # ax3.plot(pos[:,0], pos[:,1], pos[:,2], marker='.')
            # ax3.grid(True)
            # ax[ii].set_zlabel(r'$\xi_{}$'.format(3))

        ax3.set_xlabel(r'$\xi_1$')
        ax3.set_ylabel(r'$\xi_2$')
        ax3.set_zlabel(r'$\xi_3$')
        ax3.set_xlim(-0.9,0.9)
        ax3.set_ylim(-0.9,0.9)
        ax3.set_zlim(zRange[0],zRange[1])
        ax3.set_aspect('equal')


        plt.draw()
        # fig3.draw()

        time.sleep(0.1)
        # ax3.clear()


plt.show()

print('')
print('script ended. ')
print('')
