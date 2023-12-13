# -*- coding: utf-8 -*-
'''!@file       RomiEnc.py
    @brief      Romi quadrature encoder firmware
    @details    RomiEnc.py contains the class driver for the Romi motor encoders.
    @author:    Joseph Penrose & Paolo Navarro
    @date       November 14, 2023
'''
# Required modules
from pyb import Timer
import utime
class RomiEnc():
    '''!@brief      A driver class for one of Romi's quadrature encoders.
        @details    RomiEnc.py contains the class driver for the Romi motor encoders. Romi uses
                    quadrature encoders, which are decoded every pass through MainTask. RomiEnc
                    relies on the MicroPython Timer.ENC_AB counter mode, which dedicates one hardware
                    timer for one encoder. It automatically counts encoder ticks using internal
                    timer interrupts, and RomiEnc simply pulls the delta from the STM32 internal
                    registers on a regular basis and keeps track of motion data. Like all of 
                    Romi's firmware, it initializes with a set of Shares designed to pass 
                    important data in and out to the rest of Romi's program.
                    
        @details    Class initializes an encoder object, determines relative posisition of an
                    encoder, and returns the position and delta values. Romi may also zero the
                    position when needed.
    '''

    def __init__(self, counter_tim, ch1_pin, ch2_pin, ticksprev, ShareTuple):
        '''!@brief      Initializes and returns an object associated with a Romi encoder.
            @details    Encoder timer channels are initialized, and the initial position is captured.
            
            @details    z_queue is an 8-bit queue with garbage data acting as a flag.
                        If you want to request a zero on the encoder, put a flag by putting
                        anything in the Queue. It works by simply checking whether it's full
                        or not. 
                        
                        
                            ShareTuple contains three Shares. Each one is used to pass out encoder data.
                            ShareTuple[0] = position    [rad]   (float)
                            ShareTuple[1] = delta       [rad]   (float)
                            ShareTuple[2] = speed       [rad/s] (float)
                            ShareTuple[3] = zero flag   [trash] (flag)

                        
            @param      counter_tim     Timer object to use for encoder counts. Must be timer 
                                        2 or 4 on ME 405 STM32 boards.
            @param      ch1_pin         Corresponding channel 1 pin for encoder count timer.
            @param      ch2_pin         Corresponding channel 2 pin for encoder count timer.
            @param      ticksprev       Number of encoder ticks per revolution of output shaft
                                        for the motor used.
            @param      ShareTuple      A tuple containing all of RomiEnc's Shares objects.
        '''
        # Set up encoder timer channels
        self.ENC_A = counter_tim.channel(1, pin=ch1_pin, mode=Timer.ENC_AB) 
        self.ENC_B = counter_tim.channel(2, pin=ch2_pin, mode=Timer.ENC_AB)
        
        # Declare class attributes
        # Pre-allocation of "constants" speeds up update()      
        self.counter_tim = counter_tim              # store name of counter timer
        self.counter_AR = counter_tim.period()      # counter auto-reload value
        self.counter_AR_th = self.counter_AR//2     # half of AR value, for over/underflow correction
        self.upd_flag = False                       # new update flag
        self.position = 0                           # absolute position
        self.delta = 0                              # delta between updates
        self.delta_t = 1                            # delta_t between updates (set 1 to avoid div/0!)
        self.speed = 0                              # speed between updates
        self.x0 = counter_tim.counter()             # initialize x0
        self.x1 = self.x0                           # initalize x1
        self.t0 = utime.ticks_us                    # initialize t0
        self.ticks2rev = 1/ticksprev                # store rev/ticks factor
        self.ticks2rad = 2*3.14/ticksprev           # store rad/ticks factor
        
        # Set up access to encoder data Shares & Queues
        self.pos_share = ShareTuple[0]              # encoder position Share
        self.pos_share.put(0)                       # Initialize position
        self.del_share = ShareTuple[1]              # encoder delta Share
        self.del_share.put(0)                       # Initialize delta
        self.spd_share = ShareTuple[2]              # encoder speed Share
        self.spd_share.put(0)                       # Initialize speed
        self.z_queue = ShareTuple[3]                # encoder zero flag Queue
        
        
        
    def update(self):
        '''!@brief      Updates encoder position, delta, and speed
            @details    Encoder position written to the timer is set as the 'next position',
                        change in position is calculated as delta. If delta is very large,
                        we determine that the AR value was reached on the timer, so an overflow
                        calculation is made to correct the absolute position of the motor.
                        Pushes encoder data to their respective Shares.
        '''
        # Calculate encoder delta between updates
        self.x1 = self.counter_tim.counter()                        # get current encoder count
        self.delta_t = utime.ticks_diff(utime.ticks_us(), self.t0)  # get delta_t
        self.delta = self.x1 - self.x0                              # calculate delta = x1-x0
        
        # Correct over/underflow condition
        if self.delta > self.counter_AR_th or self.delta < -self.counter_AR_th:
            if self.delta < 0:                      # overflow condition
                self.delta += self.counter_AR       # correct for the overflow
            else:                                   # underflow condition
                self.delta -= self.counter_AR       # correct for the underflow

        # Store current position data
        self.position += self.delta                 # update position    
        self.x0 = self.x1                           # update previous position
        self.t0 = utime.ticks_us()                  # update previous time
        self.upd_flag = True                        # raise new update flag
        
        # Calculate speed 
        self.speed = self.delta * self.ticks2rad / (self.delta_t/10**6)     # rad/s
        
        # Push encoder data to Shares
        self.pos_share.put(self.position*self.ticks2rad)    # encoder position Share
        self.del_share.put(self.delta*self.ticks2rad)       # encoder delta Share
        self.spd_share.put(self.speed)                      # encoder speed Share
        

        
    def zero(self):
        '''!@brief      Resets the encoder position to zero
            @details    Resets self.position to zero
        '''
        # zero out position register
        self.position = 0
    
    
    
    def MainTask(self):
        '''!@brief      Main cotask task for RomiEnc.
            @details    The RomiEnc main task has states:
                
                        1:  Normal operation state. Continuously compute encoder data and push
                            all data to Shares. Zero out position data if requested.
                            
            @details    Like all of Romi's cooperative multitasking tasks, MainTask is written
                        as a generator function with an infinite loop. Each pass through the
                        program runs through the MainTask while loop once, until it reaches a
                        yield.
        ''' 
        # Remember, tasks are infinite generators
        while True:
            # Check if someone asked us to zero
            if self.z_queue.full():
                self.zero()             # zero encoder as requested
                self.z_queue.clear()    # lower the zero flag
            
            # Run encoder update
            self.update()
            
            yield 1                     # end of task