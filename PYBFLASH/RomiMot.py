# -*- coding: utf-8 -*-
'''!@file       RomiMot.py
    @brief      Romi DC motor firmware
    @details    RomiMot.py contains the class driver for the Romi DC motors.
    @author:    Joseph Penrose & Paolo Navarro
    @date       November 14, 2023
'''
# Required modules
from pyb import Timer, Pin
class RomiMot():
    '''!@brief      A driver class for one of Romi's DC motors.
        @details    RomiMot.py contains the class driver for the Romi DC motors. Romi uses DC
                    motor drivers that require a binary direction signal (forward/reverse) and
                    a PWM signal for motor effort. This firmware accepts a PWM Timer object and
                    two Pin objects from MicroPython on STM32 to send signals to a motor. It works
                    with cooperative multitasking by updating direction and effort every pass
                    through the program. Commands from Romi are taken through Shares. Like all of 
                    Romi's firmware, it initializes with a set of Shares designed to pass 
                    important data in and out to the rest of Romi's program.
                    
        @details    Class initializes a motor object, reads Romi's desired motor commands, and
                    sends the requisite signals to a motor driver.
    '''
    
    def __init__(self, PWM_tim, EFF_pin, DIR_pin, ShareTuple): 
        '''!@brief      Initializes and returns an object associated with a Romi DC motor.
            @details    Motor PWM timer is initialized and pin references are stored. It also
                        initalizes the motor direction to be enabled forward, and zero effort.
                        
                            ShareTuple contains two Shares. Each one is used to pass in commands.
                            ShareTuple[0] = enable      [bool]  (bool)
                            ShareTuple[1] = duty        [%]     (float)
                        
            @param      PWM_tim         Timer object to use for motor duty cycle PWM.
            @param      EFF_pin         A Pin object corresponding to the effort pin on the
                                        motor driver.
            @param      DIR_pin         A Pin object corresponding to the direction pin on the
                                        motor driver.
            @param      ShareTuple      A tuple containing all of RomiMot's Shares objects.
        '''
        # Set up access to motor data Shares
        self.EN_share = ShareTuple[0]           # motor enable Share
        self.EN_share.put(1)                    # Initialize enable bool
        self.duty_share = ShareTuple[1]         # motor duty cycle Share
        self.duty_share.put(0)                  # Initialize duty cycle
        
        # Create PWM object for effort control, startup 0
        self.EFF = PWM_tim.channel(1, pin=EFF_pin, mode=Timer.PWM, pulse_width_percent=0)
        
        # Make reference to direction pin and startup low (fwd)
        self.DIR = Pin(DIR_pin, mode=Pin.OUT_PP)
        self.DIR.low()
        
        # Internal enable bool, startup on
        self.EN = True



    def set_duty(self, duty): 
        '''!@brief      Set the PWM duty cycle for the DC motor.
            @details    This method sets the duty cycle to be sent to the motor driver to a
                        given percentage. Positive values cause effort in one direction (Romi
                        forward), negative values in the opposite direction (Romi backward).
                        
            @details    The duty cycle signal is clamped between positive and negative 100%.
            @param      duty    A signed number holding the duty cycle of the PWM signal sent
                                to the motor driver.
        ''' 
        # If motor enabled, set duty
        if self.EN:
            # forward motion
            if duty > 0:
                if duty > 100:
                    self.duty = 100
                self.EFF.pulse_width_percent(duty)
                self.DIR.low()
            # reverse motion
            else:
                if duty < -100:
                    self.duty = -100
                self.EFF.pulse_width_percent(-1*duty)
                self.DIR.high()
        # If motor disabled, set zero
        else:
            self.EFF.pulse_width_percent(0)
        
        
        
    def MainTask(self):
        '''!@brief      Main cotask task for RomiMot.
            @details    The RomiMot main task has states:
                
                            1:  Normal operation state. Continuously retrieve motor commands from
                                Shares and set motor signals accordingly.
                            
            @details    Like all of Romi's cooperative multitasking tasks, MainTask is written
                        as a generator function with an infinite loop. Each pass through the
                        program runs through the MainTask while loop once, until it reaches a
                        yield.
        ''' 
        # Remember, tasks are infinite generators
        while True:
            # Check enabled from Share
            self.EN = self.EN_share.get()
            
            # Control motor. aka set duty
            self.set_duty(self.duty_share.get())
            
            yield 1                     # end of task