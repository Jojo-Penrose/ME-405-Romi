# -*- coding: utf-8 -*-
'''!@file       LineSensors.py
    @brief      Romi line sensor firmware
    @details    LineSensors.py contains the class driver for the line sensors Romi uses in the
                ME 405 Mechatronics term project.
    @author     Joseph Penrose & Paolo Navarro
    @date       December 1, 2023
'''

class LineSensors():
    '''!@brief      A driver class for Romi's line sensors.
        @details    LineSensors.py contains the class driver for the line sensors Romi uses in the
                    ME 405 Mechatronics term project. The sensors are from the Pololu QTR 
                    reflectance sensor series using analog output. Each one reports a voltage from
                    0~Vdd corresponding to the amount of sensed reflectance just in front of the
                    sensor. On Romi, they are mounted below the chassis at their ideal working
                    distance and used to measure the term project maze. The track is drawn using
                    black ink on white paper, and the sensors can be used to read whether they see
                    black or white, line or paper. Like all of Romi's firmware, it initializes 
                    with a set of Shares designed to pass important data in and out to the rest
                    of Romi's program.
                    
        @details    LineSensor creates an object that runs regularly with Romi's multitasking.
                    It constantly updates the line sensor reading measurement Share.                        
    '''
    
    def __init__(self, LS_shares, Pins): 
        '''!@brief      Initializes and returns an object associated with a Romi LineSensors.
            @details    LineSensors unpacks its shares and builds all of the variables needed
                        for line sensing.
                        
                            LS_shares contains two Shares. They are used to pass out data
                            and to communicate with the main program.
                            LS_shares[0] = sensor value     [--]    (float)
                            LS_shares[1] = finish line flag [trash] (flag)
                            LS_shares[2] = sensor sum       [--]    (float)
                            
            @param      LS_shares   A tuple containing all of LineSensor's Shares objects.
            @param      Pins        A tuple containing all of the ADC objects associated with
                                    each line sensor.
        '''
        # Set up access to line sensor Shares
        self.sens_val_share = LS_shares[0]      # Final sensor value Share
        self.sens_val_share.put(0)              # Initialize sensor value
        self.finish_flag = LS_shares[1]         # finish line detect trash flag Queue
        self.sens_sum_share = LS_shares[2]      # Sensor sum Share
        self.sens_sum_share.put(0)              # Initialize sensor sum
        self.finish_flag.clear()                # clear just in case
        
        # Create ADC references
        # Axial
        self.L2 = Pins[0]           # Far left 
        self.L1 = Pins[1]           # Mid left
        self.C = Pins[2]            # Center, Arduino pin A2
        self.R1 = Pins[3]           # Mid right, Arduino pin A3
        self.R2 = Pins[4]           # Far right, Arduino pin A4
        # Front
        self.FL2 = Pins[5]          # Far left
        self.FL1 = Pins[6]          # Mid left
        self.FC = Pins[7]           # Center
        self.FR1 = Pins[8]          # Mid right
        self.FR2 = Pins[9]          # Far right
        
        # Internal variables
        self.sens_val = 0.0      # weighted sensor value
        self.Cval = 0.0             # center sensor value (fraction)
        self.allsum = 0.0           # sum of all sensors
        self.summid = 0.0           # sum of middle three sensors
        self.offline = 0.35         # sumall when seeing white
        self.midoff = 0.20         # summid when seeing white
        self.whsens = 0.065         # one sensor val when it sees white
        self.midweak = 0.60         # summid when just going into deadzone
        
        # Useful line sensing flags
        self.offline_flag = 0       # True if no sensors detecting line
        self.finish_maybe = 0       # True if all sensors detect line
        
        

    def get_sens_val(self): 
        '''!@brief      Read line sensors and calculate weighted sensor reading.
            @details    The method first reads all of the line sensors' ADC readings. Then
                        it calculates a weighted "sensor value," a signed number whose sign
                        represents left/right alignment to a guide line and whose magnitude
                        represent the distance off of center.
                        
            @details    The method also detects if the sum of all sensors falls below a certain
                        threshold, which indicates the line sensors have completely lost sight
                        of the track. It uses the previous sensor value to check whether Romi
                        has gone off left or right, and sets a constant sensor reading to
                        correct the error, equivalent to the maximum reading on either side.
        ''' 
        # # Get sensor values as a fraction of the max reading (4095)
        # L2val = self.L2.read()/4095
        # L1val = self.L1.read()/4095
        # Cval  = self.C.read() /4095
        # R1val = self.R1.read()/4095
        # R2val = self.R2.read()/4095
        
        # Get sensor values as a fraction of the max reading (4095)
        L2val = self.FL2.read()/4095
        L1val = self.FL1.read()/4095
        Cval  = self.FC.read() /4095
        R1val = self.FR1.read()/4095
        R2val = self.FR2.read()/4095
        
        # Save Cval
        self.Cval = Cval
        
        # Save sumall
        self.sumall = sum((L2val, L1val, Cval, R1val, R2val))
        self.summid = sum((L1val, Cval, R1val))
        
        # # sensval calculation
        # # Off line conditions
        # if self.summid < self.midoff and self.sens_val > 2.4:
        #     # print('off line to the right')
        #     self.sens_val = 2.5
        #     self.offline_flag = 1
        # elif self.summid < self.midoff and self.sens_val < -2.4:
        #     # print('off line to the left')
        #     self.sens_val = -2.5
        #     self.offline_flag = 1
        # # Outer edge conditions
        # elif self.summid < self.midweak and R2val > self.whsens:
        #     # print('far right mode')
        #     self.sens_val = sum((0*L2val, 0*L1val, 2*R1val, 5*R2val))
        # elif self.summid < self.midweak and L2val > self.whsens:
        #     # print('far left mode')
        #     self.sens_val = sum((-5*L2val, -2*L1val, 0*R1val, 0*R2val))
        # # Normal, on line condition
        # else:
        #     # Weighted sum. Don't add center so it adds up to zero
        #     self.sens_val = sum((-2.5*L2val, -1.0*L1val, 1.0*R1val, 2.5*R2val))
            
        # sensval calculation
        # Off line conditions
        if self.sumall < self.offline and self.sens_val > 0:
            # print('off line to the right')
            self.sens_val = 1.5
            self.offline_flag = 1
        elif self.sumall < self.offline and self.sens_val < 0:
            # print('off line to the left')
            self.sens_val = -1.5
            self.offline_flag = 1
        # Normal, on line condition
        else:
            # Weighted sum. Don't add center so it adds up to zero
            self.sens_val = sum((-2.5*L2val, -0.5*L1val, 0.5*R1val, 2.5*R2val))
            
        # If currently lost, check if the line was reacquired
        if self.offline_flag == 1 and self.sumall > self.offline:
            # print('you are back on the line')
            self.offline_flag = 0
            
        # Finish line detection
        if self.finish_flag.empty() and self.sumall > 1.7:
            # print('all sensors high. potentially on the finish line')
            self.finish_maybe = 1
            
        # Confirm finish line when sensor value falls
        if self.finish_maybe == 1 and self.sumall < self.offline:
            # print('finish line detected')
            self.finish_maybe = 0
            self.finish_flag.put(1)
            
        # print(f'L2: {L2val}; L1: {L1val}; C: {Cval}; R1: {R1val}; R2: {R2val}')
        # print(f'L2: {L2val}; summid: {self.summid}; R2: {R2val}')
        # print(f'Sum of all sensors: {self.sumall}')
        # print(f'sensor val: {self.sens_val}')
        # print(f'sensor val: {self.sens_val}; L2: {L2val}; summid: {self.summid}; R2: {R2val}')
        
        
        
    def MainTask(self):
        '''!@brief      Main cotask task for LineSensors.
            @details    The LineSensors main task has states:
                
                            1:  Normal operation state. Continuously compute sensor data and push
                                all data to Shares.
                            
            @details    Like all of Romi's cooperative multitasking tasks, MainTask is written
                        as a generator function with an infinite loop. Each pass through the
                        program runs through the MainTask while loop once, until it reaches a
                        yield.
        ''' 
        # Remember, tasks are infinite generators
        while True:
            
            # Trash flag if center sensor reads black, else clear flag
            # if self.Cval > 0.5:
            #     self.c_flag.put(1)
            # else:
            #     self.c_flag.clear()
                    
            # Read sensors and push weighted reading
            self.get_sens_val()                         # read sensors
            self.sens_val_share.put(self.sens_val)      # push weighted val to Share
            self.sens_sum_share.put(self.sumall)      # push weighted val to Share
            
            # print(f'sumall: {self.sumall}')
            
            # if self.finish_flag.full():
            #     print('~~~~~~~~~~~~~~~~~~~~~~~~~~')
            
            # DEBUG
            # print(f'Sensor reading: {sens_val}')
            # DEBUG
            
            yield 1                         # end of task