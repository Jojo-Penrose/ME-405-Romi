# -*- coding: utf-8 -*-
'''!@file       RomiMM.py
    @brief      Romi MasterMind
    @details    RomiMM.py contains the class driver for Romi's main command and control system.
    @author:    Joseph Penrose & Paolo Navarro
    @date       November 16, 2023

    N O T E S :        
        
'''
'''!@file       RomiMM.py
    @brief      Romi DC motor firmware
    @details    RomiMM.py contains the RomiMM class, which is the main command and control
                center for Romi.
    @author     Joseph Penrose & Paolo Navarro
    @date       November 16, 2023
'''
# Required modules
import utime
import math
class RomiMM():
    '''!@brief      A Romi robot MasterMind brain object.
        @details    Class creates and contains Romi MasterMind object. It contains all of
                    Romi's maneuver, mapping, and command programming.
                    
        @details    MasterMind is Romi's brain. It takes data from Romi's sensors and
                    uses it to determine its state relative to some goal. It uses that
                    state to make a best guess at what its next move should be. That is
                    a fancy way of saying Romi uses line-following or dead-reckoning to
                    navigate some set path. Before line-following, Romi will have some
                    basic motion paths written to test dead-reckoning accuracy.
                    
        @details    Mechanically, Romi uses an algorithm to convert differential drive
                    into world coordinates, determine the next velocity it needs to
                    align with the goal, and convert that velocity back into differential
                    drive.
                
        @details    Romi will always init at world 0, with world X straight ahead.
                    This is irrespective of true north or whatever. BNO will calibrate
                    to these axes on startup.
                    
        @details    Say we ask Romi to go to a world position waypoint.
                    Dead_Reck gets us where we are at every update interval.
                    When we see Romi is close enough to that waypoint, we ask to go to
                    the next waypoint.
                    Even if Romi doesn't hit waypoint exactly (HINT: IT WON'T), we can
                    get where we are going inch by inch, as close as we can.
                    
        @details    __**F I N I T E  ~  S T A T E  ~  M A C H I N E**__
        @details    Romi's states are either doing a move, or sitting still and doing a
                    thing. Not both. So we can define the states as either the non-move
                    action (calibrate BNO? pick up flag?) or a motion type (to waypoint,
                    fixed maneuver, etc).
    '''
    
    def __init__(self, dict_L, dict_R, BNO_shares, LS_shares, LidarDist, W, r, LineController): 
        '''!@brief      Initializes and returns a Romi Brain object.
            @details    RomiMM's init method creates references to essentially all of Romi's
                        Shares and Queues. MasterMind is the "central office" of Romi, so all
                        information has to pass in and out.
                        
            @details    The left and right drive dictionaries provide access to all data 
                        pertaining to each motor, encoder, and speed controller set. Currently,
                        Romi does not use closed-loop speed control. However, when an effective
                        controller can be implemented, access to the control data is available.
            
                            Shares dictionary format reference:
                            dict_L = {"Encoder": enc_L_shares,    | enc_L_shares = (pos_L, del_L, spd_L, z_enc_L)
                                      "CL Gains": gains_L_shares, | gains_L_shares = (Kp_L, Ki_L, Kd_L)
                                      "CL Signals": ctrl_L_shares,| ctrl_L_shares = (CL_R_L, CL_FB_L, CL_C_L, CL_eclr_L)
                                      "Motor": mot_L_shares}      | mot_L_shares = (mot_EN_L, duty_L)
                            example: to put 100 in Kp_L, the call is: dict_L["CL Gains"][0].put(100)
              
            @param      dict_L      A dictionary containing all of Romi's left-side drive
                                    (encoder, motor, etc) Shares.
            @param      dict_R      A dictionary containing all of Romi's right-side drive
                                    (encoder, motor, etc) Shares.
            @param      BNO_shares  A tuple containing all of BNO's Shares objects.
            @param      LS_shares   A tuple containing all of LineSensor's Shares objects.
            @param      LidarDist   A Share containing the distance sensor's distance measurement [mm].
            @param      W           Romi's wheelbase width in [m].
            @param      r           Romi's wheel radius in [m].
            @param      LineController  LineCL line following closed-loop controller object.
        '''
        # Store a reference to ALL SHARES. ULTIMATE KNOWLEDGE, ULTIMATE POWER
        self.dict_L = dict_L                    # left side drive data
        self.dict_R = dict_R                    # right side drive data
        self.BNO_phi = BNO_shares[0]            # BNO heading Share
        self.BNO_eul_x = BNO_shares[2]          # BNO compass Share
        self.BNO_cal_flag = BNO_shares[1]       # BNO cal flag Queue
        self.BNO_z_flag = BNO_shares[8]         # BNO zero flag Queue
        self.ax_sens_val_share = LS_shares[0]   # Axial sensor value Share
        self.finish_flag = LS_shares[1]         # finish line trash flag Queue
        self.LidarDist = LidarDist              # [mm] distance sensor Share
        
        # Store Romi attributes
        self.W = W              # Romi wheelbase width, on tire centers
        self.r = r              # Romi wheel radius
        
        # Build Romi's self-awareness (mapping data)
        self.X = 0.0            # [m]       X position in world coordinates
        self.Y = 0.0            # [m]       Y position in world coordinates
        self.V_c = 0.0          # [m/s]     Romi's center point SPEED SCALAR
        self.phi = 0.0          # [rad]     Romi's heading in standard angle
        self.theta = 0.0        # [rad]     Romi's delta-phi, change in angle b/w updates
        self.w_L = 0.0          # [rad/s]   Left motor angular velocity. Pulled from Encoder
        self.w_R = 0.0          # [rad/s]   Right motor angular velocity. Pulled from Encoder
        self.l_L = 0.0          # [m]       Left wheel delta
        self.l_R = 0.0          # [m]       Right wheel delta
        
        # Internal control variables
        self.state = 0          # state variable
        self.man_flag = 0       # maneuver flag
        self.d_c = 0            # little delta
        self.dist = 0.0         # maneuver distance storage
        self.halfcirc_flag = 0  # half-circle marker flag, for Lab 0x04
        
        # Closed-loop control objects
        self.LCL = LineController   # LineCL object used for line following control



    def Dead_Reck(self): 
        '''!@brief      Estimate Romi position delta
            @details    Use encoder position data to estimate the change in Romi's position
                        in world coordinates.
                        
                            ASSUME: All of Romi's motion deltas are PERFECTLY CIRCULAR OR STRAIGHT.
                                    Romi's wheel speeds DID NOT CHANGE BETWEEN UPDATES (RR SCHED!!)
                                    Romi's wheels DID NOT SLIP (WE HOPE!!!)
                        
            @details    Romi thinks in straight lines and perfect arcs only. See the full derivation
                        of the dead-reckoning system on the main project page.
                        
            @details    AVOID DIVIDE ZERO ERROR! (theta = 0)
                        Do straight line calc when theta very small. Romi uses the average of both
                        wheel deltas for straight moves to acccount for error.
        ''' 
        # Calculate theta, the change in heading
        phi_new = self.BNO_phi.get()        # Get newest heading from BNO
        phi_old = self.phi                  # Save old phi for delta calculation
        self.theta = phi_new - phi_old      # [rad] Calculate change in heading
        self.phi = phi_new                  # Store newest world heading
        
        # Ask Encoder for deltas, distance traveled by each wheel
        l_L = self.r * self.dict_L["Encoder"][1].get()  # [m] Left wheel distance
        l_R = self.r * self.dict_R["Encoder"][1].get()  # [m] Right wheel distance
        
        # Arc calculation
        if abs(self.theta) > 0.001:
            # Calculate d_c, chord length of the arc path
            # See full derivation
            self.d_c = 1 / self.theta * (l_L + l_R) * math.sin(self.theta / 2)   # [m] path chord length
            
            # Update world position with new delta
            # See full derivation
            self.X += self.d_c * math.cos(1/2*(phi_new + phi_old))
            self.Y += self.d_c * math.sin(1/2*(phi_new + phi_old))
            
        # Straight move calculation
        else:
            # Go by mean of the two encoder deltas
            self.d_c = 0.5 * (l_L + l_R)             # [m] mean distance travelled by wheels
            
            # Go by mean of the new and old phi as well
            phi_s = 0.5 * (phi_new + phi_old)   # [m] mean distance travelled by wheels
            
            # Update world position with new delta
            self.X += self.d_c * math.cos(phi_s)     # just a line!
            self.Y += self.d_c * math.sin(phi_s)     # easy!            
        
        # Update wheel speeds and deltas, for control purposes:
        self.w_L = self.dict_L["Encoder"][2].get()  # [rad/s] Left encoder angular speed
        self.w_R = self.dict_R["Encoder"][2].get()  # [rad/s] Right encoder angular speed
        self.l_L = l_L                              # [m] Left wheel delta
        self.l_L = l_R                              # [m] Right wheel delta



    def Drive(self, duty_L, duty_R): 
        '''!@brief      Romi drive command.
            @details    Helper method used to send motor efforts out to Romi's motors. Simply
                        put the desired wheel speeds in the motor Shares, and the corresponding
                        motor tasks will update the motors when it's their turn to run on the
                        scheduler.
            @param      duty_L      Target left motor speed, as a duty cycle percentage.
                                    Positive values drive forward and vice versa.
            @param      duty_R      Target right motor speed, as a duty cycle percentage.
                                    Positive values drive forward and vice versa.
        ''' 
        
        # ...and pass them to the motors!
        self.dict_L["Motor"][1].put(duty_L)            # Left motor duty
        self.dict_R["Motor"][1].put(duty_R)            # Right motor duty
        
        

    def LineMove(self, target, speed): 
        '''!@brief      Romi straight line move.
            @details    Romi moves in a straight line at a target speed for a set
                        distance. Romi will end the move when it has moved at least
                        distance ln. This move task works as a GENERATOR, a SUB-TASK within
                        a STATE in the MainTask.
            @param      target      Target distance in [m].
            @param      speed       Target forward speed, as a duty cycle percentage.
        '''                 
        # While we have not yet crossed the target distance:
        while self.dist < target:
            # Keep driving forward
            self.Drive(speed, speed)
            # Calculate distance travelled during move:
            self.dist += self.d_c   # [m] distance travelled by Romi
            yield 0                 # not done!
                
        self.man_flag = 0   # lower maneuver flag
        self.dist = 0.0     # reset distance var
        yield 1             # done!
        
        

    def Turn(self, angle): 
        '''!@brief      Romi turn move.
            @details    Romi turns on a dime through an angle. It checks which turning
                        direction is faster to get to the target heading. Then, it turns in
                        that direction until Romi is facing the target angle. This move task 
                        works as a GENERATOR, a SUB-TASK within a STATE in the MainTask.
            @param      angle       Target angle in [rad]. Signed.
        '''     
        # Calculate target
        target = self.phi + angle
        
        # Saturation: Romi reads only < 6.28 (2pi)
        if target > 6.28:
            target -= 6.28
            
        # Which way to turn?            
        if angle > 0:
            # Turn left:
            w_L = -15
            w_R = 15         
        else:
            # Turn right:
            w_L = 15
            w_R = -15
        
        # While we have not yet found the target angle:
        while abs(target - self.phi) > 0.001:
            # Keep turning
            self.Drive(w_L, w_R)
            yield 0         # not done!
                
        self.man_flag = 0   # lower maneuver flag
        yield 1             # done!
        
        
        
    def LineFollow(self, sensor, speed):
        '''!@brief      Romi line follower control algorithm.
            @details    The algorithm starts by setting both wheel speeds w_L and w_R to a small
                        value, straight line drive [duty cycle]. The sensor value is signed: negative
                        means too far right, zero means dead on, positive means too far left. The
                        algorithm uses the sensor reading to shift the duty cycle difference (add
                        curvature to the motion path) while the reading is non-zero. Like all Romi 
                        maneuvers, this function is a generator sub-task.
                        
            @details    The algorithm also uses the closed-loop line following controller to operate
                        on the sensor data and provide a better control signal. Since the target
                        value is zero (on the line), proportional control really is just using the
                        sensor value directly. The proportional gain is useful to control the strength
                        of Romi's corrections, and integral and derivative controls can be applied to
                        smooth out Romi's motions.
            @param      sensor      Weighted sensor value from the line sensors.
            @param      speed       Base speed in duty cycle percent.
        '''     
        # Initial duty cycles            
        w_L_0 = speed
        w_R_0 = speed
        
        while True:
            # Update duty cycles by weight
            sensor_val = sensor.get()
            
            CS = self.LCL.controller(0, sensor_val)
            
            # print(f'Sensor: {sensor_val}; Control: {CS}')
            # print(f'Sensor: {sensor_val}')
            # print(f' X = {self.X}; Y = {self.Y}: phi = {self.phi}')
            
            w_L = w_L_0*(1 - CS)    # signed by direction
            w_R = w_R_0*(1 + CS)
            
            self.Drive(w_L, w_R)    # send duty cycles to motors
            # self.Drive(0, 0)        # send duty cycles to motors
            
            yield 1                 # exit subtask
        
        
        
    def MainTask(self):
        '''!@brief      Main cotask task for RomiMM.
            @details    The RomiMM main task has states:
                
                            0:  Init state. Romi waits without moving until BNO has reported that
                                the sensors are calibrated and have begun transmitting data. Once
                                BNO reports it is ready, MasterMind zeros out its world coordinates
                                before kicking into the first motion state.
                            1:  ???
                            2:  ???
                            3:  ???
                            4:  ???
                            5:  ???
                            
            @details    Like all of Romi's cooperative multitasking tasks, MainTask is written
                        as a generator function with an infinite loop. Each pass through the
                        program runs through the MainTask while loop once, until it reaches a
                        yield.
        ''' 
        # Romi has some more complex init state work than other labs have had
        while self.state == 0:
            # Wait for BNO to finish calibrating        
            if self.BNO_cal_flag.full() and self.BNO_eul_x.get() != 0:
                self.BNO_cal_flag.clear()   # ack flag, lower
                self.BNO_z_flag.put(1)      # ask BNO to zero phi
                self.LCL.Kp(7)              # set controller P gain
                self.state = 1              # go to state 1
                
                # Go to Lab 0x04
                self.curr_man = self.LineFollow(self.ax_sens_val_share, 20)     # Create line follower gen
                self.man_flag = 1                           # raise maneuver flag. we got one!
                self.state = 3      # Start following line
            
            yield self.state        # exit task
        
        # Remember, tasks are infinite generators
        while True:
            # This stuff MUST RUN EVERY PASS
            # Romi must experience the world in order to conquer it
            self.Dead_Reck()    # first of all, where are we?
                        
            # State 1: Chill
            if self.state == 1:
                
                
                self.Drive(0, 0)        # quit movin

                
                yield self.state
                
            # State 2: Do Maneuver
            elif self.state == 2:
                # This is a weird-looking call but it works great. Since the maneuver is
                # a generator, this for loop says "keep calling the maneuver until it
                # finishes." And since the generator ends when we reach our goal, then
                # it breaks out when the command is done! Pretty slick!
                
                while self.man_flag:
                    next(self.curr_man)
                    print(f' X = {self.X}; Y = {self.Y}; phi = {self.phi}')
                    yield self.state
                    
                self.state = 1      # Go back to Chill
                yield self.state    # done
                    
            # State 3: Circle Follow: Lab 0x04
            elif self.state == 3:
                # Look out for when Romi has done a half-circle. When it does, stop
                # after Romi passes zero again. Circle complete!
                
                if self.man_flag:
                    next(self.curr_man)
                    print(f' X = {self.X}; Y = {self.Y}; phi = {self.phi}')
                    yield self.state
                    
                    # check for circle startpoint
                    if self.halfcirc_flag == 0 and self.X < -0.2:
                        self.halfcirc_flag = 1
                        
                    if self.halfcirc_flag == 1 and self.X > 0:
                        self.man_flag = 0
                        self.halfcirc_flag = 0
                    
                else:
                    self.state = 1      # go back to chill
                    
                    yield self.state    # done
                    
            # State 4: Do Term Project
            elif self.state == 4:
                # First, line follow until obstacle detected.
                self.curr_man = self.LineFollow(self.ax_sens_val_share, 20) # Create line follower gen
                
                # Until the wall gets within 30mm of Romi's face...
                while self.LidarDist.get() > 30:
                    # ...continue line following
                    next(self.curr_man)
                    print(f' X = {self.X}; Y = {self.Y}; phi = {self.phi}')
                    yield self.state
                
                # Next, handle obstacle avoidance.
                # Turn left 90 degrees.
                self.curr_man = self.Turn(3.14/2)           # Create turn maneuver
                self.man_flag = 1                           # raise maneuver flag. we got one!
                
                # Keep maneuvering until we've turned.
                while self.man_flag:
                    next(self.curr_man)
                    print(f' X = {self.X}; Y = {self.Y}; phi = {self.phi}')
                    yield self.state
                    
                # Go straight 200 mm (~8").
                self.curr_man = self.LineMove(0.200, 20)    # Create line maneuver
                self.man_flag = 1                           # raise maneuver flag. we got one!
                
                # Keep maneuvering until we've gone out beyond the obstacle.
                while self.man_flag:
                    next(self.curr_man)
                    print(f' X = {self.X}; Y = {self.Y}; phi = {self.phi}')
                    yield self.state
                
                # Turn back to the right 90 degrees.
                self.curr_man = self.Turn(-3.14/2)          # Create turn maneuver
                self.man_flag = 1                           # raise maneuver flag. we got one!
                
                # Keep maneuvering until we've turned.
                while self.man_flag:
                    next(self.curr_man)
                    print(f' X = {self.X}; Y = {self.Y}; phi = {self.phi}')
                    yield self.state
                    
                # Go straight 400 mm (~16").
                self.curr_man = self.LineMove(0.400, 20)    # Create line maneuver
                self.man_flag = 1                           # raise maneuver flag. we got one!
                
                # Keep maneuvering until we've passed the obstacle.
                while self.man_flag:
                    next(self.curr_man)
                    print(f' X = {self.X}; Y = {self.Y}; phi = {self.phi}')
                    yield self.state
                
                # Turn back to the right 90 degrees.
                self.curr_man = self.Turn(-3.14/2)          # Create turn maneuver
                self.man_flag = 1                           # raise maneuver flag. we got one!
                
                # Keep maneuvering until we've turned.
                while self.man_flag:
                    next(self.curr_man)
                    print(f' X = {self.X}; Y = {self.Y}; phi = {self.phi}')
                    yield self.state
                    
                # Go straight 200 mm (~8") and get back to the line.
                self.curr_man = self.LineMove(0.200, 20)    # Create line maneuver
                self.man_flag = 1                           # raise maneuver flag. we got one!
                
                # Keep maneuvering until we've reached the line again.
                while self.man_flag:
                    next(self.curr_man)
                    print(f' X = {self.X}; Y = {self.Y}; phi = {self.phi}')
                    yield self.state
                
                # Turn back to the left 90 degrees.
                self.curr_man = self.Turn(3.14/2)          # Create turn maneuver
                self.man_flag = 1                           # raise maneuver flag. we got one!
                
                # Keep maneuvering until we've turned.
                while self.man_flag:
                    next(self.curr_man)
                    print(f' X = {self.X}; Y = {self.Y}; phi = {self.phi}')
                    yield self.state
                
                # Next, resume line following until reaching the finish.
                self.curr_man = self.LineFollow(self.ax_sens_val_share, 20) # Create line follower gen
                
                # Until the finish line is detected...
                while not self.finish_flag.full():
                    # ...continue line following
                    next(self.curr_man)
                    print(f' X = {self.X}; Y = {self.Y}; phi = {self.phi}')
                    yield self.state
                    
                self.finish_flag.clear()    # clear finish line flag
                
                # Finally, return to the start.
                
                self.state = 1      # Go back to Chill
                yield self.state    # done
                
                
                
            # RED ALERT, RED ALERT, INVALID STATE VARIABLE!!!  
            else:
                raise ValueError("Invalid state variable in task RomiMM")  
                break
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        