#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sep  2 13:38:43 2017

@author: maxkwon
"""
import numpy as np
import matplotlib.pyplot as plt # plotting package

class TrapazoidalProfile:
    
    def __init__(self, max_acc, max_vel, time_):
        
        self.set_time = time_
        
        self.time_dt = 0.00001
        
        self.max_acc = max_acc
        
        self.max_vel = max_vel
        
        self.ramp_time = self.max_vel/self.max_acc
        
        self.ramp_dis = 0.5*(self.max_vel * self.ramp_time)
        
        self.iterations = time_/self.time_dt
        
        
        
    def generateProfile(self, target):
        
        positions = []
        vels = []
        times = []
        
        time = self.time_dt
        pos = 0
        vel = 0
        acc = self.max_acc
        
        last_vel = 0
        last_pos = 0
        
        counter = 0
        
        #print(int(self.iterations))
        
        while (pos <= target):
            
            time += self.time_dt
            
            ramp_time = vel/self.max_acc
            
            ramp_dis = 0.5*(vel*ramp_time)
                    
            if (target-ramp_dis <= pos):
                
                acc = -1.0 * self.max_acc
                
            elif (vel < self.max_vel):

                acc = self.max_acc
                
            else:
                
                acc = 0
            
            pos = last_pos + (vel * self.time_dt)
            last_pos = pos
            
            vel = last_vel + (acc * self.time_dt)
            last_vel = vel
                
            if (counter == int(self.iterations)):
                counter = 0
                vels.append(vel)
                positions.append(pos)
                times.append(time)
                #print(pos, vel, time, self.iterations)
            
            counter += 1
            
            
    #    plt.title("Profile")
     #   plt.plot(times, positions, 'r')
      #  plt.savefig('Profile.jpg')
        
        return positions, vels
        
#profile = TrapazoidalProfile(5, 10, .01)

#profile.generateProfile(10)
