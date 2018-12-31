#!/usr/bin/env python3

#Created on Sat Jan 13 14:06:07 2018

#@author: maxkwon


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

##Elevator is a daughter of the ControlLoop superclass
class Elevator(control_loop.ControlLoop):
    
    def __init__(self, name = "Intake") :
        
        self.stall_torque = 0.71 * 2
        # Stall Current in Amps
        self.stall_current = 134 * 2
        # Free Speed in RPM
        self.free_speed = 18730
        # Free Current in Amps
        self.free_current = 0.7 * 2
        # Resistance of the motor
        self.resistance = 12.0 / self.stall_current
        # Motor velocity constant
        self.Kv = ((self.free_speed / 60.0 * 2.0 * numpy.pi) /
                   (12.0 - self.resistance * self.free_current))
        # Torque constant
        self.Kt = self.stall_torque / self.stall_current
        # Gear ratio
        self.G = 20.0
        #radius in meters
        self.r = 0.0381
        #mass of the elevator carriage, the arm and the cube is 19 pounds (divide by 2.2 to get kg)
        self.mass = 19.0/2.2
         
        self.max_vel = (self.free_speed/self.G) * (2.0 * 3.1415 * self.r)

        
        C1 = self.Kt * self.G * self.G / (self.Kv * self.resistance * self.r * self.r * self.mass)
        C2 = self.G * self.Kt / (self.resistance * self.r * self.mass)
        
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
        q_pos = .65#.55#0.35
        q_vel = .9#2.5#1.0
        self.Q = numpy.matrix([[(1.0 / (q_pos ** 2.0)), 0.0],
                           [0.0, (1.0 / (q_vel ** 2.0))]])

        self.R = numpy.matrix([[(1.0 / (12.0 ** 2.0))]])
        self.K = controls.dlqr(self.A, self.B, self.Q, self.R)
        
        self.U_max = numpy.matrix([[12.0]])
        self.U_min = numpy.matrix([[-12.0]])
 
        self.InitializeState()
        
        print(self.K)
        
        
    def dlqr(A, B, Q, R):
      """Solves for the optimal lqr controller.
    
        x(n+1) = A * x(n) + B * u(n)
        J = sum(0, inf, x.T * Q * x + u.T * R * u)
      """
    
      # P = (A.T * P * A) - (A.T * P * B * numpy.linalg.inv(R + B.T * P *B) * (A.T * P.T * B).T + Q
      P, rcond, w, S, T = slycot.sb02od(n=A.shape[0],m=B.shape[1],A=A,B=B,Q=Q,R=R,dico='D')
    
      F = numpy.linalg.inv(R + B.T * P *B) * B.T * P * A
      return F
        
        

class IntegralElevator(Elevator):
    
    def __init__(self, name = "IntegralElevator"):
        
        super(IntegralElevator, self).__init__(name=name)

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
        
       
        q_pos = 0.25
        q_vel = 0.10
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
        
    def simulate(self, elevator, goal_pos, goal_vel,
             controller_elevator,
             t_time):
        
        if controller_elevator is None:
            controller_elevator = elevator
       
        vbat = 12.0
        
        initial_t = 0
        
        time = initial_t
            
        profile_index = 0       

        while(time < t_time):
            
            goal_data = [goal_pos[profile_index], goal_vel[profile_index]]
         
            velocity_goal = goal_vel[profile_index]
            
            U = controller_elevator.K * (goal_data - elevator.X) + ((velocity_goal/elevator.max_vel) * vbat)
            
            U[0, 0] = numpy.clip(U[0, 0], -vbat, vbat)
            self.x.append(elevator.X[0, 0])
            
            if self.v:
                last_v = self.v[-1]
            else:
                last_v = 0
                       
            self.x_hat.append(elevator.X_hat[0,0])
            self.v.append(elevator.X[1, 0])
            self.a.append((self.v[-1] - last_v) / controller_time_step)
          
            elevator.Update(U)
          
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
    
elevator = Elevator()
elevator_controller = IntegralElevator()

profiler = prof.TrapazoidalProfile(5, 2, .005)
vals = profiler.generateProfile(1)

pos = vals[0]
vels = vals[1]
    
R = numpy.matrix([[1.2], [0.0]])
simulator.simulate(elevator, pos, vels, controller_elevator=None, t_time = total_time)

simulator.Plot()

#print(profile[0])
