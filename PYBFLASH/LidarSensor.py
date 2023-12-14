# -*- coding: utf-8 -*-
'''!@file       LidarSensor.py
    @brief      Romi LIDAR distance sensor firmware
    @details    LidarSensor.py contains the class driver for the Pololu (item # 4064) LIDAR
                -based distance sensor.
    @author     Joseph Penrose & Paolo Navarro
    @date       December 5, 2023
'''
class LidarSensor():
    '''!@brief      Romi LIDAR distance sensor firmware class
        @details    LidarSensor.py contains the class driver for the Pololu (item # 4064) LIDAR
                    -based distance sensor. Because of the way interrupts work on the STM32 with
                    MicroPython, the interrupt that is used to actually measure the pulse width
                    of the sensor output is in Romi's 'main.py' and runs on rising and falling
                    edges. The pulse width is pushed to a Share during the ISR. Like all of Romi's
                    firmware, it initializes with a set of Shares designed to pass important data 
                    in and out to the rest of Romi's program.
                    
        @details    LidarSensor creates an object that runs regularly with Romi's multitasking.
                    It works with the pulse width measurement ISR to constantly update the
                    distance measurement Share.
    '''
    
    def __init__(self, LidarShares):
        '''!@brief      Initializes and returns an object associated with a Romi LidarSensor.
            @details    Unpacks Shares and initializes variables.
        
                            LidarShares contains two Shares. They are used to pass data around Romi's main program.
                            LidarShares[0] = distance       [mm]    (float)
                            LidarShares[1] = pulse width    [us]    (float)
            
            @param      LidarShares A tuple containing all of LidarSensor's Shares objects.
        '''
        self.distance = LidarShares[0]      # distance measurement Share    
        self.dt = LidarShares[1]            # pulse width Share
        self.distance.put(999)              # Distance in [mm]
        self.d_calc = 999                   # calculated distance [mm]
        
        

    def getDistance(self):
        '''!@brief      Helper function that measures the distance sensed by the LIDAR sensor.
            @details    TThe returned distance is in millimeters. The calculation can be found 
                        in Pololu's documentation for the sensor on their website (item # 4064).
        '''
        self.d_calc = 3/4*(self.dt.get()-1000) # [mm]
            
        
        
    def MainTask(self):
        '''!@brief      Main cotask task for LidarSensor.
            @details    The LidarSensor main task has states:
                
                            1:  Normal operation state. Continuously compute distance and push the
                                current measurement to the distance Share.
                            
            @details    Like all of Romi's cooperative multitasking tasks, MainTask is written
                        as a generator function with an infinite loop. Each pass through the
                        program runs through the MainTask while loop once, until it reaches a
                        yield.
        ''' 
        while True:
            # State 1: Get distance
            self.getDistance()
            self.distance.put(self.d_calc)
            # print(f'distance {self.d_calc} mm')
            yield 'grra'
            