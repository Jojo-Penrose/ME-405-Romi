# -*- coding: utf-8 -*-
'''!@file       LineCL.py
    @brief      Romi closed-loop controller designed for line following.
    @details    LineCL is a special closed-loop controller that is specifically set up for
                Romi to use during line following. It is derived from an old class that was
                used in previous labs for motor control. Previously, this class had a task
                that ran in multitasking, but this class is designed to be called as a helper
                function from within Romi's main program. The basic design of the PID
                controller is derived from the ME 305 Intro to Mechatronics term project lab.
    @author     Joseph Penrose & Paolo Navarro
    @date       October 17, 2023
'''
# Required modules
import utime
class LineCL():
    '''!@brief      Romi line following closed-loop controller object
        @details    LineCL is a special closed-loop controller that is specifically set up for
                    Romi to use during line following. It is derived from an old class that was
                    used in previous labs for motor control. Previously, this class had a task
                    that ran in multitasking, but this class is designed to be called as a helper
                    function from within Romi's main program. The basic design of the PID
                    controller is derived from the ME 305 Intro to Mechatronics term project lab.
                    
        @details    The LineCL class is intended to be used inside of Romi's main program to
                    compute differential drive effort required to stay on track following the
                    line of the ME 405 Mechatronics term project maze.
    '''

    def __init__(self, Kp, Ki = 0, Kd = 0, highSat = None, lowSat = None):
        '''!@brief      Constructs a closed loop controller object
            @details    Constructs a CL, P/PI/PD/PID controller with optional control signal saturation.
                        The type of controller is set by the non-zero gains entered. The init method
                        builds all of the necessary variables from the inputs, and sets up signal
                        saturation if it was requested. Signal saturation clamps the object's
                        output level to between the two limits.
            @param      Kp      Proportional gain, required for all control types
            @param      Ki      Integral gain, required for PI/PID control only. Default off
            @param      Kd      Derivative gain, required for PD/PID control only. Default off
            @param      highSat Upper limit of control signal saturation level (i.e. PWM 100%). Default off
            @param      lowSat  Lower limit of control signal saturation level (i.e. PWM -100%). Default off
        '''
        # Store gains in memory
        if Kp <= 0:
            raise Exception("Proportional gain must be positive & non-zero")
        else:
            self.Kp = Kp    # Proportional gain
        self.Ki = Ki        # Integral gain
        self.Kd = Kd        # Derivative gain
        self.err = 0.0      # Error double
        self.esum = 0.0     # Error sum double
        self.t0 = utime.ticks_us()                                       # Curr utime (us)
        self.dt = utime.ticks_diff(utime.ticks_us(), self.t0)/(10**6)     # Delta time (s)
        self.Cp = 0.0       # Proportional control signal
        self.Ci = 0.0       # Integral control signal
        self.Cd = 0.0       # Derivative control signal
        self.C = 0.0        # Controller output signal
        
        # Set up control signal saturation
        if highSat is not None and lowSat is not None:
            self.highSat = highSat  # Record high level saturation
            self.lowSat = lowSat    # Record low level saturation
            self.satEn = True       # Raise saturation enabled flag
        elif highSat is not None and lowSat is None:
            raise Exception("Control saturation requires both upper and lower limits to be defined")
        else:
            self.satEn = False      # Lower saturation enabled flag
        
        
    def ChangeKp(self, newKp):    
        '''!@brief      Update proportional gain Kp with a new value
            @details    Update proportional gain Kp with a new value
            @param      newKp   New value for proportional gain
        '''   
        # Update Kp
        if newKp <= 0:
            raise Exception("Proportional gain must be positive & non-zero")
        else:
            self.Kp = newKp
        
        
    def ChangeKi(self, newKi):    
        '''!@brief      Update integral gain Ki with a new value
            @details    Update integral gain Ki with a new value. Set to 0 to
                        turn off integral control.
            @param      newKi   New value for integral gain
        '''   
        # Update Ki
        if newKi < 0:
            raise Exception("Integral gain must be non-negative")
        else:
            self.Ki = newKi
        
        
    def ChangeKd(self, newKd):    
        '''!@brief      Update derivative gain Kd with a new value
            @details    Update derivative gain Kd with a new value. Set to 0 to
                        turn off derivative control.
            @param      newKd   New value for derivative gain
        '''   
        # Update Kd
        if newKd < 0:
            raise Exception("Derivative gain must be non-negative")
        else:
            self.Kd = newKd
        

    def controller(self, R, FB) :
        '''!@brief      Closed-loop, negative feedback controller
            @details    Runs closed-loop negative feedback control in P, PI, PD, or 
                        PID control. Accepts reference input R and feedback source 
                        FB and computes control signal using the gains set in the 
                        initialization. Must use numerical inputs. Optionally performs
                        control signal saturation if it was defined in initialization.
                        
                        Basic PID operation derived from ME 305 Intro to Mechatronics
                        
                        Input validation code written by Daniel Goldberg, stackoverflow
            @param      R       Reference input. Numerical input only
            @param      FB      Feedback source. Numerical input only
        '''   
        # Validate inputs
        try:
            float(R)
            float(FB)           # Only int/float data types work in float()
        except ValueError:
            print("Numeric (int or float) inputs only")
        
        # Compute error
        self.err = R - FB       # Error at the summing junction (NFB)
        
        # Deal with time
        self.dt = utime.ticks_diff(utime.ticks_us(), self.t0)/(10**6)   # Delta time (s)
        self.t0 = utime.ticks_us()                                      # New time (us)           
        
        # Compute proportional control
        self.Cp = self.Kp*self.err                  # Proportional control is just an error gain
    
        # Compute integral control
        interr = self.err*self.dt
        self.esum += interr
        self.Ci = self.Ki*self.esum                 # Integral control using Riemann sum
    
        # Compute derivative control
        self.Cd = self.Kd*(self.err/self.dt)        # Derivative control using high school calculus approx
        
        # Add up control signals
        self.C = self.Cp + self.Ci + self.Cd        # Control is sum of signals
        
        # Handle control signal saturation
        if self.satEn:                              # Check if saturation enabled
            if self.C > self.highSat:
                self.C = self.highSat               # High level saturation
            if self.C < self.lowSat:
                self.C = self.lowSat                # Low level saturation
        
        # Send control signal
        return self.C                               # Return control signal