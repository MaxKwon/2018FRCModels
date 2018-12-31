#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jan 11 19:04:54 2018

@author: maxkwon
"""


import numpy
from matplotlib import pylab
import slycot 
import controls
import control_loop
import UnitTest.profile as prof

#dt in seconds
controller_time_step = .005 #10 milliseconds
sim_time_step = .000001

#time in seconds
total_time = 10

class Intake(control_loop.ControlLoop):
    
    def __init__(self, name = "Intake") :
        
        self.stall_torque = 0.71
        # Stall Current in Amps
        self.stall_current = 134
        # Free Speed in RPM
        self.free_speed = 18730
        # Free Current in Amps
        self.free_current = 0.7
        # Resistance of the motor
        self.R = 12.0 / self.stall_current
        # Motor velocity constant
        self.Kv = ((self.free_speed / 60.0 * 2.0 * numpy.pi) / (12.0 - self.R * self.free_current))
        # Torque constant
        self.Kt = self.stall_torque / self.stall_current
        # Gear ratio
        self.G = 831.0 #(56.0 / 12.0) * (54.0 / 14.0) * (64.0 / 18.0) * (48.0 / 16.0)
        # Moment of inertia, measured in CAD.
        # Extra mass to compensate for friction is added on.
        #2720 pound in
        self.J = 1.0 #0.34 + 0.40
        
        self.max_vel = (18730.0/831.0) * 2.0 * 3.1415
        
        
        C1 = self.G * self.G * self.Kt / (self.R  * self.J * self.Kv)
        C2 = self.Kt * self.G / (self.J * self.R)

        
        self.A_continuous = numpy.matrix(
        [[0, 1],
         [0, -C1]])

        # Start with the unmodified input
        self.B_continuous = numpy.matrix(
                [[0],
                [C2]])
    
        self.C = numpy.matrix([[1, 0]])
        self.D = numpy.matrix([[0]])
        
        
        self.A, self.B = self.ContinuousToDiscrete(
        self.A_continuous, self.B_continuous, controller_time_step)
        
        
        #LQR
        q_pos = 0.5 #.15
        q_vel = 20.0 # 1.0
        self.Q = numpy.matrix([[(1.0 / (q_pos ** 2.0)), 0.0],
                           [0.0, (1.0 / (q_vel ** 2.0))]])
        self.R = numpy.matrix([[(1.0 / (12.0 ** 2.0))]])
        
        self.K = controls.dlqr(self.A, self.B, self.Q, self.R)
        
        self.U_max = numpy.matrix([[12.0]])
        self.U_min = numpy.matrix([[-12.0]])
        
  #      self.X = numpy.zeros((self.A.shape[0], 1))
  #      self.Y = self.C * self.X
  #      self.X_hat = numpy.zeros((self.A.shape[0], 1))
        
        print(self.K)
  
        self.InitializeState()
        
        
    def dlqr(A, B, Q, R):
      """Solves for the optimal lqr controller.
    
        x(n+1) = A * x(n) + B * u(n)
        J = sum(0, inf, x.T * Q * x + u.T * R * u)
      """
    
      # P = (A.T * P * A) - (A.T * P * B * numpy.linalg.inv(R + B.T * P *B) * (A.T * P.T * B).T + Q
      P, rcond, w, S, T = slycot.sb02od(n=A.shape[0],m=B.shape[1],A=A,B=B,Q=Q,R=R,dico='D')
    
      F = numpy.linalg.inv(R + B.T * P *B) * B.T * P * A
      return F
        
        

class IntegralIntake(Intake):
    
    def __init__(self, name = "IntegralIntake"):
        
        super(IntegralIntake, self).__init__(name=name)

        self.A_continuous_unaugmented = self.A_continuous
        self.B_continuous_unaugmented = self.B_continuous

        self.A_continuous = numpy.matrix(numpy.zeros((3, 3)))
        self.A_continuous[0:2, 0:2] = self.A_continuous_unaugmented
        self.A_continuous[0:2, 2] = self.B_continuous_unaugmented

        self.B_continuous = numpy.matrix(numpy.zeros((3, 1)))
        self.B_continuous[0:2, 0] = self.B_continuous_unaugmented

        self.C_unaugmented = self.C
        self.C = numpy.matrix(numpy.zeros((1, 3)))
        self.C[0:1, 0:2] = self.C_unaugmented

        self.A, self.B = self.ContinuousToDiscrete(
        self.A_continuous, self.B_continuous, controller_time_step)

        q_pos = 0.12
        q_vel = 2.00
        q_voltage = 4.0
        self.Q = numpy.matrix([[(q_pos ** 2.0), 0.0, 0.0],
                           [0.0, (q_vel ** 2.0), 0.0],
                           [0.0, 0.0, (q_voltage ** 2.0)]])

        r_pos = 0.05
        self.R = numpy.matrix([[(r_pos ** 2.0)]])

        self.K_unaugmented = self.K
        self.K = numpy.matrix(numpy.zeros((1, 3)))
        self.K[0, 0:2] = self.K_unaugmented
        self.K[0, 2] = 1
              
        self.X = numpy.zeros((self.A.shape[0], 1))
        self.Y = self.C * self.X
        self.X_hat = numpy.zeros((self.A.shape[0], 1))
        
        self.InitializeState()


class Simulator(object):
    
    def __init__(self):
        self.t = []
        self.x = []
        self.v = []
        self.a = []
        self.x_hat = []
        self.u = []
        self.offset = []
        
    def simulate(self, intake, goal_pos, goal_vels,
             controller_intake,
             t_time):
        
        if controller_intake is None:
            controller_intake = intake
      
        vbat = 12.0
        
        initial_t = 0
        time = initial_t
            
        profile_index = 0
        
        #print(goal[0,1])
        
        while(time < t_time):
            
            goal_data = [goal_pos[profile_index], goal_vels[profile_index]]
            #real_goal = numpy.matrix([[goal_data],[0.0]])
            
           # X_hat = intake.X
           
            velocity_goal = goal_vels[profile_index]
                
          #  print(velocity_goal)
            
            U = controller_intake.K * (goal_data - intake.X) + ((velocity_goal/intake.max_vel) * vbat)
            
            U[0, 0] = numpy.clip(U[0, 0], -vbat, vbat)
            self.x.append(intake.X[0, 0])
            
            if self.v:
                last_v = self.v[-1]
            else:
                last_v = 0
                
                
            self.x_hat.append(intake.X_hat[0,0])
            self.v.append(intake.X[1, 0])
            self.a.append((self.v[-1] - last_v) / controller_time_step)
          
            intake.Update(U)
          
            self.t.append(time)
            self.u.append(U[0, 0])
            
            if (profile_index < len(goal_pos) - 1):
                profile_index += 1
            
            time += controller_time_step
            
            
    def Plot(self):
        
        pylab.subplot(3, 1, 1)
        pylab.plot(self.t, self.x, label='x')
        pylab.plot(self.t, self.x_hat, label='x_hat')
        pylab.legend()
    
        pylab.subplot(3, 1, 2)
        pylab.plot(self.t, self.u, label='u')
       # pylab.plot(self.t, self.offset, label='voltage_offset')
        pylab.legend()
    
        pylab.subplot(3, 1, 3)
        pylab.plot(self.t, self.a, label='a')
        pylab.legend()
    
        pylab.show()
        
        

    
print("oh god")
    
simulator = Simulator()
    
intake = Intake()
intake_controller = IntegralIntake()

profiler = prof.TrapazoidalProfile(5, .5, .01) # was 20, 20
pos, vels = profiler.generateProfile(.7)

#print(profile)
    
R = numpy.matrix([[.7], [0.0]])
simulator.simulate(intake, goal_pos=pos, goal_vels = vels, controller_intake=None, t_time = total_time)

simulator.Plot()

#print(profile[0])
            
                
            
            

        
        
            
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
