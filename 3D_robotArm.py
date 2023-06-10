#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np
import math as mth
import matplotlib.pyplot as plt
import os
import time 
from vedo import *
from os import path

def getTransformation(t, phi, rot):

    if phi == -1:  # no rotation
        R = np.eye(3)

    else:
        
        c = np.cos(phi * np.pi / 180)
        s = np.sin(phi * np.pi / 180)
        
        if rot == 'x':
          R = np.array(
              [[1, 0, 0],
              [0, c, -s],
              [0, s, c]]
          )
        elif rot == 'y':
          R = np.array(
              [[c, 0, s],
              [0, 1, 0],
              [-s, 0, c]]
          )
        else: # rot = 'z'
          R = np.array(
              [[c, -s, 0],
              [s, c, 0],
              [0, 0, 1]]
          )

    T = np.block([[R, t],
                  [np.zeros((1, 3)), 1]])

    return T

def kinematic(phi1,phi2, phi3, phi4):
    phi = np.array([phi1, phi2, phi3, phi4])
    L1 = 6
    L2 = 9
    L3 = 5
    L4 = 4
    p1 = np.array([[0],[0],[0]])
    p2 = np.array([[0],[0],[L1]])
    p3 = np.array([[0],[0],[L2]])
    p4 = np.array([[0],[0],[L3]])
    p5 = np.array([[0],[0],[L4]])
    
    # Local transformation matrices 
    
     
    T_01 = getTransformation(p1, phi[0],'z')
    T1_2 = getTransformation(p2, phi[1],'y')
    T2_3 = getTransformation(p3, phi[2],'y')
    T3_4 = getTransformation(p4, phi[3],'y')
    T4_5 = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, L4],
        [0, 0, 0, 1]
    ])

    #Local to global
    T_02 = T_01 @ T1_2
    T_03 = T_02 @ T2_3
    T_04 = T_03 @ T3_4
    T_05 = T_04 @ T4_5

    # Define the coordinates of first joint and create sphere
    p1 = [0,0,0]
    s1=Sphere(p1, r=0.5, c='black')

    # Joint 2
    p2 = T_02[:3,3]
    s2=Sphere(p2, r=5, c='purple')

    # Joint 3
    p3 = T_03[:3,3]
    s3=Sphere(p3, r=3, c='purple') 

    # Joint 4
    p4 = T_04[:3,3]
    s4=Sphere(p4, r=2, c='black') 

    # Joint 5
    p5 = T_05[:3,3]
    s5=Sphere(p5, r=2, c='purple') 

    

    #Create cylinder arms
    c1 = Cylinder(pos=[p1,p2],r=5, axis=(0, 0, 1), cap=True, res=24, c='purple',alpha=1)

    c2 = Cylinder(pos=[p2,p3],r=3, axis=(0, 0, 1), cap=True, res=24, c='purple',alpha=1)

    c3 = Cylinder(pos=[p3,p4],r=2,axis=(0, 0, 1), cap=True, res=24, c='black',alpha=1)

    c4 = Cylinder(pos=[p4,p5],r=1, axis=(0, 0, 1), cap=True, res=24, c='light grey',alpha=1)
    arm = s1 + s2 + s3 + s4+ s5 + c1 + c2 + c3 + c4
    return s1,s2,s3, s4,s5, c1, c2, c3, c4
    
# function to make the arm move
def loop_func(event):
    global s1,s2, s3, s4,s5, c1, c2, c3, c4, arm, phi
    sa,sb,sc, sd, se, ca, cb, cc, cd = kinematic(0.0, 0.0, phi, phi)
    s1.points(sa.points())
    s2.points(sb.points())
    s3.points(sc.points())
    s4.points(sd.points())
    s5.points(se.points())
    c1.points(ca.points())
    c2.points(cb.points())
    c3.points(cc.points())
    c4.points(cd.points())
    arm = s1 + s2 + s3 + s4+ s5 + c1 + c2 + c3 + c4

    #Update scene
    plt.render()
    time.sleep(0.1)
    video.add_frame()
    
    #Update phi
    phi += np.pi/20
    


phi = np.pi/20

# Video file
video = Video("tmp.mp4", 
              backend='ffmpeg', 
              fps = 24
             ) 

#Start position
s1,s2,s3, s4,s5, c1, c2, c3, c4 = kinematic(0.0, 0.0, 0.0, 0.0)
arm = s1 + s2 + s3 + s4+ s5 + c1 + c2 + c3 + c4

plt = Plotter(size=(1050, 600))
plt += [ arm, __doc__]
plt.background("black", "w").add_global_axes(axtype=1).look_at(plane='xy')


plt.add_callback("timer", loop_func)
plt.timer_callback("create", dt=50)
plt.show().close()

# merge all the recorded frames
video.close()                        

# Convert the video file spider.mp4 to play on a wider range of video players 
if path.exists("./animation.mp4"):
    os.system("rm animation.mp4")
    
os.system("ffmpeg -i tmp.mp4 -pix_fmt yuv420p animation.mp4")
os.system("rm tmp.mp4")

