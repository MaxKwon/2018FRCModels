#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Aug 17 00:34:32 2017

@author: maxkwon
"""

import numpy as np
import matplotlib.pyplot as plt # plotting package
import profile as prof

#dt in seconds
controller_time_step = .01 #10 milliseconds
sim_time_step = .000001

#time in seconds
total_time = 10

class GearPickup:
    
    def __init__(self):
        # Stall Torque in N m
        self.stall_torque = 0.71
        # Stall Current in Amps
        self.stall_current = 134.0
        # Free Speed in RPM
        self.free_speed = 18730.0
        # Free Current in Amps
        self.free_current = 0.7
        #gear ratio
        self.G = 20.0
        #Moment of Inertia
        self.J = .41
        # Resistance of the motor
        self.R = .0895
        # Motor velocity constant
        self.Kv = 164.0
        # Torque constant
        self.Kt = .00529
        
    def getAcceleration(self, voltage, velocity):
      
       acc = (((-self.G**2 * self.Kt * velocity)/(self.J * self.Kv * self.R))) + (((self.Kt * self.G  * voltage)/(self.J * self.R)))
        
       return acc  
   
    def getConstants(self):
        
        print("Kv: ", self.Kv)
        print("Kt: ", self.Kt)
        
        

class ControlSimulator:
    
    def __init__(self, GearPickup):
        
        self.gearPickup = GearPickup
        
        #saturated max and min voltage outputs to motor
        self.max_saturation = 12
        self.min_saturation = -12
        
        #PID Gains set here
        self.Kp = 2
        self.Ki = 0
        self.Kd = .15
        
    #main control loop that runs the PID controller    
    def controlLoop(self, sim_time_step, controller_time_step, total_time, profile):
        
        time = 0
        
        position = 0
        velocity = 0
        
        #output to the motor
        output_voltage = 0
        
        error = 0
        last_error = 0
        
        #collection of the position of the appendage along with the time it was there for graphing
        positions = []
        times = []
        
        i = 0
        d =0
    
        #set the first target to the first reference in the generated motion profile
        ref = profile[0]
        
        index = 0
        
        print(len(profile))
        
        #control loop
        while (time <= total_time):
            
            ref = profile[index]
            
            error = ref - position 
            
            i += error
            d = error - last_error
            
            P = self.Kp * error
            I = self.Ki * i
            D = self.Kd * d
            
            output_voltage = P + I + D 
    
            if (output_voltage > self.max_saturation):
                output_voltage = self.max_saturation
                
            if (output_voltage < self.min_saturation):
                output_voltage = self.min_saturation
                
            positions.append(position)
            times.append(time)
            
            last_error = error
            
            time += controller_time_step
            
            outputs = self.simulate(controller_time_step, sim_time_step, output_voltage, velocity)
            position += outputs[0] 
            velocity = outputs[1]
            
            if (index < (len(profile) - 1)):
                index += 1
            
        plt.title("Position")
        plt.plot(times, positions, 'r')
        plt.savefig('UnitTest.jpg')
        
    #integrator that solves for position and velocity from acceleration getter method
    def simulate(self, total_dt, sim_dt, voltage, velocity_init):
        
        time = 0
        
        acceleration = 0
        velocity = velocity_init
        position = 0
        
        outputs = [0,0]
        
        while (time <= total_dt):
            
            acceleration = self.gearPickup.getAcceleration(voltage, velocity)
            
            velocity += acceleration * sim_dt
            position += velocity * sim_dt
            
            time += sim_dt
            
        outputs = [position, velocity]
            
        return outputs
    
profiler = prof.TrapazoidalProfile(.1, 5, .01)

profile = profiler.generateProfile(.7)

gearPickup = GearPickup()

gearPickup.getConstants()

simulation = ControlSimulator(gearPickup)

simulation.controlLoop(sim_time_step, controller_time_step, total_time, profile)
    

