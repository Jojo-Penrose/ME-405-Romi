# -*- coding: utf-8 -*-
'''!@file       BNO.py
    @brief      Romi inertial measurement unit firmware
    @details    BNO.py contains the class driver for the BNO055 intertial measurement
                unit.
    @author     Joseph Penrose & Paolo Navarro
    @date       November 14, 2023
'''
from machine import I2C
import struct

class BNO():
    '''!@brief      A driver class for Romi's IMU, BNO.
        @details    BNO.py contains the class driver for the BNO055 intertial measurement
                    unit. It checks if a calibration file exists already on the robot's
                    flash memory, or else it triggers the calibration sequence and creates
                    one. Like all of Romi's firmware, it initializes with a set of Shares
                    designed to pass important data in and out to the rest of Romi's program.
                    
        @details    Since BNO communicates with Romi over I2C, it also initializes with an
                    I2C object already set up for communication.
                    
        @details    Class initializes a BNO object, handles calibration, communivation, and
                    regularly updates Shares with IMU data.
    '''
    
    def __init__(self, BNO_shares, i2c): 
        '''!@brief      Initializes and returns an object associated with a BNO IMU.
            @details    BNO unpacks its Shares and creates variables to use for data. It also
                        creates variables containing all of BNO's relevant memory addresses,
                        for reference.
                        
                            BNO_shares contains nine Shares. They are used to pass out data
                            and to communicate with the main program.
                            BNO_shares[0] = heading     [rad]   (float)
                            BNO_shares[1] = calibration complete(flag)
                            BNO_shares[2] = eul_x       [rad]   (float)
                            BNO_shares[3] = eul_y       [rad]   (float)
                            BNO_shares[4] = eul_z       [rad]   (float)
                            BNO_shares[5] = x_ang_vel   [rad/s] (float)
                            BNO_shares[6] = y_ang_vel   [rad/s] (float)
                            BNO_shares[7] = z_ang_vel   [rad/s] (float)
                            BNO_shares[8] = req zero heading    (flag)

            @param      BNO_shares  A tuple containing all of BNO's Shares objects.
            @param      i2c         An I2C object already set up to communicate with BNO.
        '''
        
        # Set up access to BNO Shares
        self.phi_share = BNO_shares[0]              # IMU heading Share
        self.phi_share.put(0)                       # Initialize heading
        self.cal_complete_flag = BNO_shares[1]      # calibration trash flag Queue
        self.BNO_z_flag = BNO_shares[8]             # zeroing flag for phi


        # Euler angles (shares)
        self.BNO_eul_x = BNO_shares[2]
        self.BNO_eul_x.put(0)                       # Initialize
        self.BNO_eul_y = BNO_shares[3]
        self.BNO_eul_y.put(0)                       # Initialize
        self.BNO_eul_z = BNO_shares[4]
        self.BNO_eul_z.put(0)                       # Initialize

        # Angular velocities (shares)
        self.BNO_xav = BNO_shares[5]
        self.BNO_xav.put(0)                       # Initialize
        self.BNO_yav = BNO_shares[6]
        self.BNO_yav.put(0)                       # Initialize
        self.BNO_zav = BNO_shares[7]
        self.BNO_zav.put(0)                       # Initialize
        
        # Internal variables
            # Addresses
        self.addr = 0x28
        self.pwr_mode = 0x3E
        self.opr_mode = 0x3D
        self.axis_remap_config = 0x41
        self.axis_sign_config = 0x42
        
            # Register Pairs [LSB ,  MSB]
        self.head =          [0x1A, 0x1B]
        self.eul_x_reg =     [0x1A, 0x1B]
        self.eul_y_reg =     [0x1C, 0x1D]
        self.eul_z_reg =     [0x1E, 0x1F]
        
        self.vel_x =         [0x14, 0x15]
        self.vel_y =         [0x16, 0x17]
        self.vel_z =         [0x18, 0x19]
        
        self.c_acc_x =       [0x55, 0x56] # Calibration coefficients
        self.c_acc_y =       [0x57, 0x58]
        self.c_acc_z =       [0x59, 0x5A]

        self.c_mag_x =       [0x5B, 0x5C]
        self.c_mag_y =       [0x5D, 0x5E]
        self.c_mag_z =       [0x5F, 0x60]

        self.c_gyr_x =       [0x61, 0x62]
        self.c_gyr_y =       [0x63, 0x64]
        self.c_gyr_z =       [0x65, 0x66]

        self.acc_radius =    [0x67, 0x68]
        self.mag_radius =    [0x69, 0x6A]
            # Misc
        self.phi = 0.0      # [rad]     heading  
        self.euler_x = 0.0
        self.euler_y = 0.0
        self.euler_z = 0.0
        self.zangle = 0     # [rad]     phi zero offset
        
        self.state = 1 # To next state
        
        # Set up i2c class
        self.bno = i2c
        
        # Initiate BNO
        self.set_opmode('CONFIGMODE') # Set operating mode
        self.bno.writeto_mem(self.addr,self.pwr_mode,b'\x00') # Set power mode
        self.bno.writeto_mem(self.addr,self.axis_remap_config,b'\x24') #remap axes (vectors)
        self.bno.writeto_mem(self.addr,self.axis_sign_config,b'\x05') #remap axes (signs)

        
    def set_opmode(self, op_mode):
        '''!@brief      A helper function used to set the operation mode (OPMODE) of the
                        BNO. A call to this function sends the required command to BNO
                        over I2C.
            @details    This method recieves a string that matches one of the five fusion 
                        modes of the BNO055 detailed in the data sheet. The specified mode
                        is keyed using a dict, and the correct register value is written
                        to the operating mode configuration register using the read_register()
                        method.
            @param      op_mode     A string used to choose the corresponding OPMODE code
                                    from a dictionary.
        '''
        self.key = {'CONFIGMODE':b'\x00', 'IMU':b'\x08', 'COMPASS': b'\x09', 'M4G':b'\x0A' , 'NDOF_FMC_OFF':b'\x0B' , 'NDOF':b'\x0C'}
        self.bno.writeto_mem(self.addr,self.opr_mode,self.key[op_mode])


    def calibration_status(self):
        '''!@brief      A helper function used to decode the current system calibration codes.
            @details    This method reads the calibration status register and
                        returns the status for each sensor by decoding the 8-
                        bit register and parsing the corresponding status bits.
            @return     sys_stat    Current 'System' calibration state.
            @return     gyr_stat    Current 'Gyrometer' calibration state.
            @return     acc_stat    Current 'Accelerometer' calibration state.
            @return     mag_stat    Current 'Magenetometer' calibration state.
        '''
        buf = bytearray()
        status = {'00':0, '01':1, '10':2, '11':3} # create dict key
        regi_data = self.bno.readfrom_mem(self.addr,0x35,1) # read register
        buf[0:1] = regi_data # put byte into buffer
        bits = f"{bin(buf[0])[2:]:0>{8}}" # turn byte into bits (str obj w/ length of 8)
        # System
        sys_stat = status[(bits[0]+bits[1])] # key value of the desired bits to corresponding bit value
        # Gyroscope
        gyr_stat = status[(bits[2]+bits[3])] # key value of the desired bits to corresponding bit value
        # Accelerometer
        acc_stat = status[(bits[4]+bits[5])] # key value of the desired bits to corresponding bit value

        # Magnetometer
        mag_stat = status[(bits[6]+bits[7])] # key value of the desired bits to corresponding bit value
            
        return sys_stat,gyr_stat,acc_stat,mag_stat #returns ints of the values
        
        
    def read_register(self, reg_pair, mode=0, mult=1): 
        '''!@brief      A helper function a to read two 8-bit registers and return the 
                        concatenated 2-byte value.
            @details    This method allows for the user to read the MSB and LSB
                        byte registers and concatenates them. The bytes are stored
                        in a buffer which is indexed and bitshifted accordingly. 
                        The output can be the raw bytearray, unsigned integer, or 
                        signed integer. For convenience, the return value can
                        be multiplied by a factor to account for units.
            @param      reg_pair    A tuple with the registers in big endian
                                    order. 
            @param      mode        An integer that selects the output mode. 
            @param      mult        A float that optionally multiplies the
                                    output by some specified factor.   
            @return     result      Depending on the selected mode, the result
                                    can be 2 hex values, an unsigned 2-byte 
                                    integer, or a signed 2-byte integer.
        '''
        #read 8 bits of data, mode=0:, 
        buf = bytearray()
        lsb = self.bno.readfrom_mem(self.addr,reg_pair[0],1)
        msb = self.bno.readfrom_mem(self.addr,reg_pair[1],1)
        
        buf[0:1] = msb
        buf[1:2] = lsb
       
        if mode == 0: # output is combined 2-bit integer value | for standard reading
            result = (buf[0] << 8)|buf[1]
            
        if mode == 1: # output is msb,lsb as hex values | for calibration coeffs
            
            result = (hex(buf[0]),hex(buf[1]))
            
        if mode == 2: # output is combined 2-bit integer with converted units
            
            result = (buf[0] << 8)|buf[1]
            result=result*mult
            
        if mode == 3: # output is SIGNED 2-bit integer with converted units
            result = struct.unpack('>1h',buf)
            result = int(result[0])
            result = result*mult
        return result


    def zero_phi(self):
        '''!@brief      A helper function used to zero out the phi parameter (Romi heading)
            @details    zangle is the z-offset used to calculate Romi's heading. On startup,
                        Romi assumes that the initial position is X,Y (0,0) and heading 0.
                        BNO only speaks in offset from what it measures to be true north.
                        zangle is then the difference between true north and the initial
                        heading, and just needs to be equal to the IMU euler x angle at the
                        desired zero position.
        '''
        self.zangle = self.euler_x
        
    def MainTask(self):
        '''!@brief      Main cotask task for BNO.
            @details    The BNO main task has states:
                
                            1:  Check for calibration file state. This state seeks
                                out a calibration coefficients text file and if
                                there is no file, go to state 2. If there is a 
                                file go to state 4.
                            2:  Recalibration state. Continuously read the
                                calibration status register and print the status
                                for each sensor on screen. Once all four bit-pairs
                                are fully calibrated, go to state 3.
                            3:  Write new calibration coefficients state. After
                                calibration, read the calibration coefficients
                                for each sensor and write them to a new text file
                                as a string of hex values. Raise the calibrated
                                flag and go to state 5.
                            4:  Write coefficients from existing file state. If
                                a calibration coefficient file exists, decode the
                                hex values and write the corresponding bytes to
                                each respective register. Afterwards, go to state 5.
                            5:  Normal operation state. Continuously read IMU registers and push
                                data to corresponding Shares. Performs a short calculation for phi
                                which accounts for zeroing out the IMU euler x angle.
             
            @details    Like all of Romi's cooperative multitasking tasks, MainTask is written
                        as a generator function with an infinite loop. Each pass through the
                        program runs through the MainTask while loop once, until it reaches a
                        yield.
        ''' 
        # Remember, tasks are infinite generators
        cal_data_new=[0,0,0,0]

        while True:
            # State 1: Check for calibration state
            if self.state == 1:
                self.set_opmode('NDOF') #set operating mode
                
                try:
                    with open('IMU_cal_coeffs.txt','r') as file: # try opening an existing file
                        pass
                
                except OSError:  # no file found
                    print('BNO requires calibration.\n')
                    print('Initiating calibration procedure.')
                    self.state = 2  # go to 'do calibration' state
                except:
                    print("Unidentified error occured in BNO")
                else:  # file was found
                    print('IMU_cal_coeffs.txt found!')
                    self.state = 4  # go to 'write existing file' state
                    #self.state = 2 # D E L E T E M E (force calibration)
                
                yield 'joe'  # end of task
                
            # State 2: Do calibration    
            elif self.state == 2:
                cali_count = 0
                
                cal_data = self.calibration_status()
                
                for i in range(4):
                    if cal_data[i] != cal_data_new[i]:
                        print('System:',cal_data[0],'Gyroscope:',cal_data[1],'Accelerometer:',cal_data[2],'Magnetometer:',cal_data[3])
                        cal_data_new[i] = cal_data[i]
                
                for i in range(4):                
                    if cal_data[i] == 3:
                        cali_count += 1
                        
                if cali_count == 4:
                    print('\n Calibration Complete! \n')
                    self.state = 3
    
                else:
                    yield 'ligma'  # yield instance of calibration  

        
                yield 'mama'                        # end of task
        
            # State 3: Write new calibration coefficients
            elif self.state == 3:  
                self.set_opmode('CONFIGMODE')
                acc_x = self.read_register(self.c_acc_x,mode = 1)
                acc_y = self.read_register(self.c_acc_y,mode = 1)
                acc_z = self.read_register(self.c_acc_z,mode = 1)
                
                mag_x = self.read_register(self.c_mag_x,mode = 1)
                mag_y = self.read_register(self.c_mag_y,mode = 1)
                mag_z = self.read_register(self.c_mag_z,mode = 1)
                
                gyr_x = self.read_register(self.c_gyr_x,mode = 1)
                gyr_y = self.read_register(self.c_gyr_y,mode = 1)
                gyr_z = self.read_register(self.c_gyr_z,mode = 1)
                
                acc_rad = self.read_register(self.acc_radius,mode = 1)
                mag_rad = self.read_register(self.mag_radius,mode = 1)
                
                data = mag_rad + acc_rad + gyr_z + gyr_y + gyr_x + mag_z + mag_y + mag_x + acc_z + acc_y + acc_x
                
                # create a new file and write calibration data
                
                with open('IMU_cal_coeffs.txt','w') as file:
                    data = str(data)
                    data = data.replace("'","")
                    data = data[1:]
                    data = data[:-1]
                    file.write(data)
                print('New calibration coefficients have been saved as "IMU_cal_coeffs.txt"\n')
                
                self.set_opmode('NDOF')
                
                self.cal_complete_flag.put(1)   # raise cal complete flag
                self.state = 5
                
                yield 'kombucha'  # end of task
            
            
            # State 4: Write existing calibration coefficients
            elif self.state == 4:
                with open('IMU_cal_coeffs.txt','r') as file:
                    data = file.read()
                    data = data.split(", ")
                            
                    for i in range(len(data)):
                        data[i] = bytes([int(data[i])])
                     
                    reg_addr = 0x6A
                    
                    for n in range(len(data)):
                        self.bno.writeto_mem(self.addr,reg_addr,data[n])
                        reg_addr -= 1
                        
                self.cal_complete_flag.put(1)
                self.state = 5
                
                yield 'beekeeping'  # End of task
            
            # State 5: Push necessesary data to shares
            elif self.state == 5:
                # Heading (phi)
                
                if self.BNO_z_flag.full():
                    self.BNO_z_flag.clear()
                    
                    self.zero_phi()
                    print('BNO: phi is zeroed')
                    
                # Phi calculation
                # Account for phi zero and saturation so that phi measures 0~2pi
                if self.euler_x > self.zangle:
                    self.phi = self.zangle - self.euler_x + 6.28    # get phi
                else:
                    self.phi = self.zangle - self.euler_x           # get phi
                self.phi_share.put(self.phi)                        # push phi to Share   
                
                # Euler Angles
                self.euler_x = self.read_register(self.eul_x_reg,mode=2,mult=(2*3.141572/5760))
                self.BNO_eul_x.put(self.euler_x)
                                
                self.euler_y = self.read_register(self.eul_y_reg,mode=2,mult=(2*3.141572/65535))
                self.BNO_eul_y.put(self.euler_y)
                
                self.euler_z = self.read_register(self.eul_z_reg,mode=2,mult=(2*3.141572/65535))
                self.BNO_eul_z.put(self.euler_z)
                
                # Angular Velocities
                self.xav = self.read_register(self.vel_x,mode=3,mult=(1/900))
                self.BNO_xav.put(self.xav)
                
                self.yav = self.read_register(self.vel_y,mode=3,mult=(1/900))
                self.BNO_yav.put(self.yav)
                
                self.zav = self.read_register(self.vel_z,mode=3,mult=(1/900))
                self.BNO_zav.put(self.zav)
                
                # print(f'eul_X: {self.euler_x}; eul_Y: {self.euler_y}; eul_Z: {self.euler_z}; phi: {self.phi}')
                # print(f'zangle: {self.zangle}')

                yield 'youfedup'
                
            # RED ALERT, RED ALERT, INVALID STATE VARIABLE!!!  
            else:
                raise ValueError("Invalid state variable in task BNO")  
                break
            
            
            
            
            
            
            
            
            
            