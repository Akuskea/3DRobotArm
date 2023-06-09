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


# In[73]:
def kinematicStart(phi1, phi2, phi3, phi4):
    L1 = 3
    #Change the L2 to see the full arm movement
    L2 = 60

    L3 = 5
    L4 = 4
    T_01 = np.array([[np.cos(phi1), -np.sin(phi1), 0.0, 0.0],
                     [np.sin(phi1), np.cos(phi1), 0.0, 0.0],
                     [0.0, 0.0, 1.0, 0.0],
                     [0.0, 0.0, 0.0, 1.0]])

    T1_2 = np.array([[np.cos(phi2), -np.sin(phi2), 0.0, 0.0],
                     [np.sin(phi2), np.cos(phi2), 0.0, 0.0],
                     [0.0, 0.0, 1.0, L1],
                     [0.0, 0.0, 0.0, 1.0]])

    T2_3 = np.array([[np.cos(phi3), -np.sin(phi3), 0.0, 0.0],
                     [np.sin(phi3), np.cos(phi3), 0.0, 0.0],
                     [0.0, 0.0, 1.0, L2],
                     [0.0, 0.0, 0.0, 1.0]])

    T3_4 = np.array([[np.cos(phi4), -np.sin(phi4), 0.0, 0.0],
                     [np.sin(phi4), np.cos(phi4), 0.0, 0.0],
                     [0.0, 0.0, 1.0, L3],
                     [0.0, 0.0, 0.0, 1.0]])

    T4_5 = np.array([[1.0, 0.0 ,0.0, 0.0],
                     [0.0, 1.0, 0.0, 0.0],
                     [0.0, 0.0, 1.0, L4],
                     [0.0, 0.0, 0.0, 1.0]])

    #Local to global
    T_02 = T_01 @ T1_2
    T_03 = T_01 @ T1_2 @ T2_3
    T_04 = T_01 @ T1_2 @ T2_3 @ T3_4
    T_05 = T_01 @ T1_2 @ T2_3 @ T3_4 @ T4_5

    # Define the coordinates of first joint and create sphere
    p1 = np.array([0.0, 0.0, 0.0, 1.0])
    p1 = p1[:3]
    s1=Sphere(p1, r=0.5, c='black') 

    # Calculate the global coordinates of joint 2 and create sphere
    j2_local = np.array([0.0, 0.0, L1, 1])
    p2 = T_01 @ j2_local
    p2 = p2[:3]
    s2=Sphere(p2, r=1, c='white') 

    # Calculate the global coordinates of joint 3 and create sphere
    j3_local = np.array([0.0, 0.0, L2, 1])
    p3 = T_02 @ j3_local
    p3 = p3[:3]
    s3=Sphere(p3, r=1, c='white') 

    # Calculate the global coordinates of joint 4 and create sphere
    j4_local = np.array([0.0, 0.0, L3, 1])
    p4 = T_04 @ j4_local
    p4 = p4[:3]
    s4=Sphere(p4, r=1, c='white') 

    # Calculate the global coordinates of joint 5 and create sphere
    j5_local = np.array([0.0, 0.0, L4, 1])
    p5 = T_05 @ j5_local
    p5 = p5[:3]
    s5=Sphere(p5, r=0.5, c='black') 
    
    
    #Create cylinder arms
    c1 = Cylinder(pos=[p1,p2],r=3, axis=(0, 0, 1), cap=True, res=24, c='grey',alpha=1)

    c2 = Cylinder(pos=[p2,p3],r=1, axis=(0, 0, 1), cap=True, res=24, c='grey',alpha=1)

    c3 = Cylinder(pos=[p3,p4],r=1, axis=(0, 0, 1), cap=True, res=24, c='grey',alpha=1)

    c4 = Cylinder(pos=[p4,p5],r=1, axis=(0, 0, 1), cap=True, res=24, c='grey',alpha=1)

    return s1, s2, s3, s4,s5, c1, c2, c3, c4
    

def kinematic(phi1, phi2, phi3, phi4):
    L1 = 3
    L2 = 8
    L3 = 5
    L4 = 4
    T_01 = np.array([[np.cos(phi1), -np.sin(phi1), 0.0, 0.0],
                     [np.sin(phi1), np.cos(phi1), 0.0, 0.0],
                     [0.0, 0.0, 1.0, 0.0],
                     [0.0, 0.0, 0.0, 1.0]])

    T1_2 = np.array([[np.cos(phi2), 0.0 ,np.sin(phi2), 0.0],
                     [-np.sin(phi2), 1.0, np.cos(phi2), 0.0],
                     [0.0, 0.0, 1.0, L1],
                     [0.0, 0.0, 0.0, 1.0]])

    T2_3 = np.array([[np.cos(phi3), 0.0 ,np.sin(phi3), 0.0],
                     [-np.sin(phi3), 1.0, np.cos(phi3), 0.0],
                     [0.0, 0.0, 1.0, L2],
                     [0.0, 0.0, 0.0, 1.0]])

    T3_4 = np.array([[np.cos(phi4), 0.0 ,np.sin(phi4), 0.0],
                     [-np.sin(phi4), 1.0, np.cos(phi4), 0.0],
                     [0.0, 0.0, 1.0, L3],
                     [0.0, 0.0, 0.0, 1.0]])

    T4_5 = np.array([[1.0, 0.0 ,0.0, 0.0],
                     [0.0, 1.0, 0.0, 0.0],
                     [0.0, 0.0, 1.0, L4],
                     [0.0, 0.0, 0.0, 1.0]])

    #Local to global
    T_02 = T_01 @ T1_2
    T_03 = T_02 @ T2_3
    T_04 = T_03 @ T3_4
    T_05 = T_04 @ T4_5
    

    # Define the coordinates of first joint and create sphere
    p1 = np.array([0.0, 0.0, 0.0, 1.0])
    p1 = p1[:3]
    s1=Sphere(p1, r=0.5, c='black') 

    # Calculate the global coordinates of joint 2 and create sphere
    j2_local = np.array([0.0, 0.0, L1, 1])
    p2 = T_01 @ j2_local
    p2 = p2[:3]
    s2=Sphere(p2, r=1, c='white') 

    # Calculate the global coordinates of joint 3 and create sphere
    j3_local = np.array([0.0, 0.0, L2, 1])
    p3 = T_02 @ j3_local
    p3 = p3[:3]
    s3=Sphere(p3, r=1, c='white') 

    # Calculate the global coordinates of joint 4 and create sphere
    j4_local = np.array([0.0, 0.0, L3, 1])
    p4 = T_03 @ j4_local
    p4 = p4[:3]
    s4=Sphere(p4, r=1, c='white') 

    # Calculate the global coordinates of joint 5 and create sphere
    j5_local = np.array([0.0, 0.0, L4, 1])
    p5 = T_04 @ j5_local
    p5 = p5[:3]
    s5=Sphere(p5, r=0.5, c='black', alpha=1) 
    

    #Create cylinder arms
    c1 = Cylinder(pos=[p1,p2],r=3, axis=(0, 0, 1), cap=True, res=24, c='light grey',alpha=1)

    c2 = Cylinder(pos=[p2,p3],r=1, axis=(0, 0, 1), cap=True, res=24, c='light grey',alpha=1)

    c3 = Cylinder(pos=[p3,p4],r=1,axis=(0, 0, 1), cap=True, res=24, c='light grey',alpha=1)

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

#Video file
video = Video("tmp.mp4", 
              backend='ffmpeg', 
              fps = 24
             ) 

#Start position
s1,s2,s3, s4,s5, c1, c2, c3, c4 = kinematicStart(0.0, 0.0, 0.0, 0.0)
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

