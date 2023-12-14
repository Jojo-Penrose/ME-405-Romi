# -*- coding: utf-8 -*-
'''!@file       main.py
    @brief      Romi main file
    @details    main.py contains the main python file that runs on the Nucleo for the ME 405
                Mechatronics Romi term project. It handles the creation of all objects, variables,
                tasks, and interrupts. It sets up all hardware connections and data Shares/Queues.
                Finally, the main game loop is simply a while try/except loop running the round-robin
                task scheduler and waiting for a KeyboardInterrupt to end the program.
                
    @details    The program flow should more or less run round-robin like this:
                    
                    Encoder:        read current pos, delta, speed, and .put() each
                    BNO:            read current alignment data and .put() each
                    LidarSensor:    check for upcoming wall and .put() distance measurement
                    LineSensors:    read line sensors, compute weighted reading, and .put() that
                    Motor:          read control signal from Shares and set_duty
                    MasterMind:     compute encoder dead reckoning, read IMU for current heading,
                                    do a best guess for current state, and compute next state
                                    including what to do with each motor.
                            
    @details    IF ROMI'S TASKS **ALWAYS** RUN IN ROUND-ROBIN, THEN DELTA FROM EVERY
                ENCODER UPDATE CAN BE USED IN MASTERMIND FOR DEAD RECKONING EVERY TIME
                NO ZEROING REQUIRED
    @author:    Joseph Penrose & Paolo Navarro
    @date       November 14, 2023   
        
'''
# Imports and startup system commands
# Sytem stuff:
import pyb, micropython
from pyb import Pin, Timer, ADC
from pyb import ExtInt
from machine import I2C
import utime

# Multitasking stuff:
from task_share import Share, Queue
import cotask

# Romi stuff:
from RomiEnc import RomiEnc
from BNO import BNO
from LineSensors import LineSensors
from LidarSensor import LidarSensor
from LineCL import LineCL
from RomiMot import RomiMot
from RomiMM import RomiMM

# Verbose exceptions:
micropython.alloc_emergency_exception_buf(100)  # please verbose exceptions



def BlueButtonCB(line):
    global mot_EN_L, mot_EN_R
    if mot_L.EN and mot_R.EN:
        mot_EN_L.put(0)
        mot_EN_R.put(0)
    elif not mot_L.EN and not mot_R.EN:
        mot_EN_L.put(1)
        mot_EN_R.put(1)
    


def DistInt(line):
    global pin_PC0, Dist_t0

    if pin_PC0.value() == 1:
        # means we got a rising edge
        Dist_t0 = utime.ticks_us()
        
    else:
        # means falling edge            
        Lidar_dt.put(utime.ticks_diff(utime.ticks_us(), Dist_t0), in_ISR = True)



if __name__ == '__main__': 
    
    ''' Set up serial ports '''  
    # Set up communication w/ debug REPL but not REPL
    vcp = pyb.USB_VCP()
    vcp.init()
    
    # BNO IMU I2C
    I2C_BNO = I2C(1, freq=200000)
    ''' End serial set up '''
    
    
    
    ''' Begin data Shares & Queues setup '''
    ''' BNO Inertial Measurement Unit '''
    # IMU:
    BNO_phi = Share('f')            # heading
    BNO_cali_flag = Queue('B', 1)   # trash flag, raise when calibration done
    BNO_z_flag = Queue('B',1)       # trash flag, raise to zero out phi w/ curr eul_x
    # Euler angles        
    BNO_eul_x = Share('f')
    BNO_eul_y = Share('f')
    BNO_eul_z = Share('f')
    # Angular velocities
    BNO_xav = Share('f')
    BNO_yav = Share('f')
    BNO_zav = Share('f')
    BNO_shares = (BNO_phi, BNO_cali_flag, BNO_eul_x, BNO_eul_y, BNO_eul_z, BNO_xav, BNO_yav, BNO_zav, BNO_z_flag)
    
    ''' Lidar Sensor '''
    #Lidar Sensor:
    distance = Share('f')
    Lidar_dt = Share('f')
    LidarShares = (distance, Lidar_dt)
    
    ''' Line Sensors '''
    # Line Sensors:
    sens_val_share = Share('f')         # Final sensor value Share
    finish_flag = Queue('B', 1)         # trash flag for finish line detection
    sens_sum_share = Share('f')         # Sensor sum Share
    LS_shares = (sens_val_share, finish_flag, sens_sum_share)

    ''' Drive L '''
    # Encoder:
    pos_L = Share('f')  # encoder position
    del_L = Share('f')  # encoder delta
    spd_L = Share('f')  # encoder speed
    z_enc_L = Queue('B', 1)     # zero encoder flag
    enc_L_shares = (pos_L, del_L, spd_L, z_enc_L)   # encoder A share tuple
    
    # Controller gains:
    Kp_L = Share('f')   # Kp gain
    Ki_L = Share('f')   # Ki gain
    Kd_L = Share('f')   # Kd gain
    gains_L_shares = (Kp_L, Ki_L, Kd_L)     # controller A gain share tuple
    
    # Controller signals:
    CL_R_L = Share('f')     # controller setpoint
    CL_FB_L = Share('f')    # controller feedback
    CL_C_L = Share('f')     # controller output signal
    CL_eclr_L = Queue('B', 1)   # zero error flag
    ctrl_L_shares = (CL_R_L, CL_FB_L, CL_C_L, CL_eclr_L)    # controller A signal share tuple
    
    # Motor signals:
    mot_EN_L = Share('B')   # motor enable flag
    duty_L = Share('f')     # motor duty cycle (signed!)
    mot_L_shares = (mot_EN_L, duty_L)   # encoder A share tuple
    
    # Put all of this stuff into a huge, Drive L dictionary
    dict_L = {"Encoder": enc_L_shares,
              "CL Gains": gains_L_shares,
              "CL Signals": ctrl_L_shares,
              "Motor": mot_L_shares}
       
    ''' Drive R '''
    # Encoder:
    pos_R = Share('f')  # encoder position
    del_R = Share('f')  # encoder delta
    spd_R = Share('f')  # encoder speed
    z_enc_R = Queue('B', 1) # zero encoder flag
    enc_R_shares = (pos_R, del_R, spd_R, z_enc_R)   # encoder B share tuple
    
    # Controller gains:
    Kp_R = Share('f')   # Kp gain
    Ki_R = Share('f')   # Ki gain
    Kd_R = Share('f')   # Kd gain
    gains_R_shares = (Kp_R, Ki_R, Kd_R)     # controller B gain share tuple
    
    # Controller signals:
    CL_R_R = Share('f')     # controller setpoint
    CL_FB_R = Share('f')    # controller feedback
    CL_C_R = Share('f')     # controller output signal
    CL_eclr_R = Queue('B', 1)   # zero error flag
    ctrl_R_shares = (CL_R_R, CL_FB_R, CL_C_R, CL_eclr_R)   # controller B signal share tuple
    
    # Motor signals:
    mot_EN_R = Share('B')   # motor enable flag
    duty_R = Share('f')     # motor duty cycle (signed!)
    mot_R_shares = (mot_EN_R, duty_R)   # encoder B share tuple
    
    # Put all of this stuff into a huge, Drive R dictionary
    dict_R = {"Encoder": enc_R_shares,
              "CL Gains": gains_R_shares,
              "CL Signals": ctrl_R_shares,
              "Motor": mot_R_shares}
    ''' End data Shares & Queues setup '''
    
    
    
    ''' Initialize firmware objects '''
    AR = 65535          # standard auto-reload value
    PS = 0              # prescaler = 0: count every tick
    ticksprev = 12*120  # number of encoder ticks per motor revolution
    W = 0.141           # [m] Romi wheelbase width, on centers
    r = 0.035           # [m] Romi wheel radius w/ tire
    
    # Set up motor encoder counter timers
    enc_L_tim = Timer(2, period = AR, prescaler = PS)
    enc_R_tim = Timer(4, period = AR, prescaler = PS)
    
    # Initialize encoder objects
    enc_L = RomiEnc(enc_L_tim, Pin.cpu.A5, Pin.cpu.B3, ticksprev, enc_L_shares)
    enc_R = RomiEnc(enc_R_tim, Pin.cpu.B6, Pin.cpu.B7, ticksprev, enc_R_shares)
    
    # Initialize BNO
    BNO = BNO(BNO_shares, I2C_BNO)
    
    # Pin object for lidar
    pin_PC0 = Pin(Pin.cpu.C0, mode=Pin.IN)

    # Create Lidar object
    Dist_t0 = 0         # Lidar interrupt t0 gloabl
    Dist_dt = 0         # Lidar interrupt dt gloabl
    Lidar = LidarSensor(LidarShares)
    
    # Initialize line sensors
    # Set up Pin objects for axial sensors
    pin_L2 = Pin(Pin.cpu.C2, mode=Pin.ANALOG) 	# Far left
    pin_L1 = Pin(Pin.cpu.C3, mode=Pin.ANALOG) 	# Mid left
    pin_C = Pin(Pin.cpu.A4, mode=Pin.ANALOG)  	# Center, Arduino pin A2
    pin_R1 = Pin(Pin.cpu.B0, mode=Pin.ANALOG) 	# Mid right, Arduino pin A3
    pin_R2 = Pin(Pin.cpu.C1, mode=Pin.ANALOG) 	# Far right, Arduino pin A4
    # Set up Pin objects for front sensors
    pin_FL2 = Pin(Pin.cpu.A6, mode=Pin.ANALOG) 	# Far left
    pin_FL1 = Pin(Pin.cpu.A7, mode=Pin.ANALOG) 	# Mid left
    pin_FC = Pin(Pin.cpu.C4, mode=Pin.ANALOG)  	# Center
    pin_FR1 = Pin(Pin.cpu.B1, mode=Pin.ANALOG) 	# Mid right
    pin_FR2 = Pin(Pin.cpu.C5, mode=Pin.ANALOG) 	# Far right
    
    # ADCs for axial sensor pins
    sens_L2 = ADC(pin_L2)
    sens_L1 = ADC(pin_L1)
    sens_C = ADC(pin_C)
    sens_R1 = ADC(pin_R1)
    sens_R2 = ADC(pin_R2)
    Pins = (sens_L2, sens_L1, sens_C, sens_R1, sens_R2)
    # ADCs for front sensor pins
    sens_FL2 = ADC(pin_FL2)
    sens_FL1 = ADC(pin_FL1)
    sens_FC = ADC(pin_FC)
    sens_FR1 = ADC(pin_FR1)
    sens_FR2 = ADC(pin_FR2)
    Pins = (sens_L2, sens_L1, sens_C, sens_R1, sens_R2, sens_FL2, sens_FL1, sens_FC, sens_FR1, sens_FR2)
    
    # Create LineSensor object
    LineSensors = LineSensors(LS_shares, Pins)
    
    # Set up timers for motor control 
    mot_L_tim = Timer(3, freq = 20_000)
    mot_R_tim = Timer(5, freq = 20_000)
    
    # Create an motor driver object
    mot_L = RomiMot(mot_L_tim, Pin.cpu.B4, Pin.cpu.B5, mot_L_shares)
    mot_R = RomiMot(mot_R_tim, Pin.cpu.A0, Pin.cpu.A1, mot_R_shares)
    
    # Finally, construct Romi's BRAIN!!!
    LineController = LineCL(1)
    MM = RomiMM(dict_L, dict_R, BNO_shares, LS_shares, distance, W, r, LineController)
    
    # Create lidar pulse width measurement interrupt
    lidar_int = ExtInt(Pin.cpu.C0, ExtInt.IRQ_RISING_FALLING, Pin.PULL_NONE, DistInt)
    
    # Create blue button motor override interrupt
    blue_int = ExtInt(Pin.cpu.C13, ExtInt.IRQ_FALLING, Pin.PULL_NONE, BlueButtonCB)
    
    ''' End firmware initialization '''
    
    
    
    ''' Multitasking with cotask '''
    # Create tasks from each firware object
    # Encoders:
    enc_L_task = cotask.Task(enc_L.MainTask, name='L Encoder', priority = 1, period=10)     # create Task object
    cotask.task_list.append(enc_L_task)                                                 # append task to scheduler
    enc_R_task = cotask.Task(enc_R.MainTask, name='R Encoder', priority = 1, period=10)     # create Task object
    cotask.task_list.append(enc_R_task)                                                 # append task to scheduler
    
    # BNO:
    BNO_task = cotask.Task(BNO.MainTask, name='BNO', priority = 1, period=10)               # create Task object
    cotask.task_list.append(BNO_task)                                                   # append task to scheduler
    
    # Lidar sensor:
    Lidar_task = cotask.Task(Lidar.MainTask, name='Lidar', priority = 1, period=10)
    cotask.task_list.append(Lidar_task)  
    
    # Line sensors:
    LS_task = cotask.Task(LineSensors.MainTask, name='LineSensors', priority = 1, period=10)# create Task object
    cotask.task_list.append(LS_task)                                                    # append task to scheduler
    
    # Motors:
    mot_L_task = cotask.Task(mot_L.MainTask, name='L Motor', priority = 1, period=10)       # create Task object
    cotask.task_list.append(mot_L_task)                                                 # append task to scheduler
    mot_R_task = cotask.Task(mot_R.MainTask, name='R Motor', priority = 1, period=10)       # create Task object
    cotask.task_list.append(mot_R_task)                                                 # append task to scheduler
    
    # MasterMind:
    MM_task = cotask.Task(MM.MainTask, name='MasterMind', priority = 1, period=10)          # create Task object
    cotask.task_list.append(MM_task)                                                    # append task to scheduler
    ''' End multitasking setup '''
    
    
    
    '''
        EYEGOR!!! THROW THE FIRST SWITCH!!!!
    '''
    
    
    
    print('Romi lives!')
    while True:
        try:                
            
            cotask.task_list.rr_sched()     # run the scheduler forever!
    
        except KeyboardInterrupt:
            break       # unless the user says otherwise!