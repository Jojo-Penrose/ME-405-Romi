'''!@file mainpage.py
    @author  Joseph Penrose & Paolo Navarro
    @brief   Markdown file for building documentation w/ Doxygen
    @mainpage
    
    @section ss_intro Introduction & Challenge
    The ME 405 Romi Term Project library contains the source code for Joseph Penrose & Paolo
    Navarro's ME 405 Mechatronics term project. The project is built on the Pololu Romi robot
    kit, the ME 405 Mechatronics Python code library written by Dr. JR Ridgely, and MicroPython.
    
    The ME 405 Mechatronics term project is designed to challenge the students' ability to write
    real-time, microcontroller-based programming integrating multiple sensors and actuators at
    once. The game board is a table-sized white paper with a maze drawn on it with black ink and
    3/4" wide lines, that may include turns, dashes, and cross-hatches. Romi must follow the lines
    from the start to the finish. There is also an obstacle, an 8" by 8" by 8" cube, somewhere
    on the track that Romi must avoid. Once Romi reaches the finish line, it must return to the 
    start by any means.
    
    [example track]
    
    Students are responsible for any mechanical design, sensor choice and integration, and 
    programming required to complete the challenge.
    
    [romi photo]
    
    @section ss_strat Strategy
    
    @subsection ss_line Line Following
    Romi uses an array of Pololu QTR reflectance sensors with analog output for line detection.
    The sensors produce an analog signal corresponding to the reflectance of the surface in
    view. For our purposes, the sensors produce voltages on the order of 0.2V for white paper
    and 2.3V for black lines (about 0.1V below Vdd). Romi's main line sensors are located on the
    front of the chassis, spaced [~~~~~] apart. Romi reads each sensor and multiplies each
    value by a weight, corresponding to the side and distance from center. Sensors on Romi's
    left are negative and vice versa. Sensors further from center are weighted higher. Then,
    summing all sensor readings together produces a signed number whose sign indicates Romi's
    direction off-center and whose magnitude grows as Romi gets further off the line. Then,
    in Romi's line-following routine, the sensor value is used with a closed-loop controller
    targeting a zero value (zero being on center). The output of this controller is used to
    differentially adjust the speed of each wheel. If Romi determines the line is to the right,
    then the left wheel speeds up and the right wheel slows down. This causes Romi to veer
    off of the straight path and turn slightly right, and tries to realign the track to the
    center of the chassis. If the line is to the right, the opposite occurs. In addition,
    Romi keeps track of the *unsigned* sum of all sensors as a check. If Romi is completely off
    of the line and all sensors lose signal, then it appears to the line-follower that Romi
    is centered because the weighted sensor value is near zero. However, the unsigned sum is
    used to determine if no sensors are reading line, and if so, Romi can determine if it has
    completely lost the track or not. If Romi determines that it has lost sight of the track,
    it applies a constant reading to the line-following controller corresponding to the last
    non-zero value to determine direction.
    
    [line sensor image]
    
    @subsection ss_obst Obstacle Detection
    Romi uses a LIDAR-based distance sensor from Pololu (item # 4064) for obstacle detection.
    The sensor outputs a pulse-width modulation (PWM) signal, whose high-side pulse-width
    can be used to calculate the measured distance. Romi uses a hardware interrupt triggered
    by both rising- and falling-edges in order to handle the pulse measurement. Then, a task
    runs regularly to capture the last pulse-width measurement and calculate the most recent
    distance reading. The sensor has a useful working distance of 50cm according to Pololu,
    but tests using the lab lighting conditions and obstacle materials showed that we could
    reliably measure distance from the obstacele from about 20cm to 10mm. Romi simply keeps
    an eye on this distance value and throws up a flag when the obstacle is spotted a short
    distance straight ahead. We trust that the approx. 15 degree viewing angle is tight
    enough to only detect the obstacle when it is straight ahead on the track, and not from
    an inappropriate position somewhere else on the track. When the obstacle is detected,
    Romi runs through a hard-coded avoidance path. The size of the obstacle is known, and
    this is a rules-accepted method of passing the obstacle.
    
    [distance sensor image]
    
    @subsection ss_fnsh Finish Line Detection
    The track's start and finish lines are distinct from the rest of the track because they
    are the only line pattern on the track where the line abtruptly ends and a horizontal
    line crosses completely across the width of the robot. This is another use of the 
    unsigned sensor value, which is used to detect if all sensors go off at once. If they do,
    and then abruptly all go dark, Romi assumes it has crossed the finish line.
    
    @subsection ss_dead Dead Reckoning and World Position
    Romi uses a dead reckoning system that blends IMU and encoder data to constantly record
    Romi's position in world coordinates, where the initial position on startup is considered
    (X,Y) (0,0) and heading phi = 0. Romi considers the direction straight ahead as +X, and to
    the left through the center of the wheel axes as +Y. Heading phi is considered a rotation
    about the Z-axis in standard coordinates, where the direction of the +X-axis is 0 and the
    +Y-axis is pi/2 rad, etc. Romi resets at 2pi rad when crossing back over the +X-axis.
    
    The dead reckoning system works on three key assumptions:
    <ul>
        <li> For a small timestep, Romi takes perfectly circular or perfectly straight paths.
        <li> For a small timestep, Romi's wheel speeds remain constant.
        <li> Romi's tires do not slip.
    </ul>
    The first two assumptions are based on the differential-drive system. If both wheels move
    at the same speed, then Romi moves in a straight line. If the wheels have different speeds,
    then Romi's path can be represented by two arcs corresponding by the individual paths taken
    by each wheel. For a small timestep, if the wheel speeds are constant, then they will trace
    perfectly circular arcs on the ground, and geometry can be used to find the distance and
    direction between start and end points.
    
    [derivation & images]
    
    @section ss_comp Component Overview
    
    @subsection ss_romi Romi
    Romi is a small, differential-drive robot kit produced by Pololu, an American educational
    electronics and robotics company. Romi's chassis is circular, roughly 6" in diameter, and
    driven by two DC gearmotors on 70mm wheels. It is balanced on front and rear ball casters.
    Pololu provides the bare bones of the robot: the chassis, wheels, motors with encoders,
    and a power mainboard that interfaces with 6x AA batteries with connections.
    
    @subsection ss_mcu Microcontroller: STM32 Nucleo & The Shoe Of Brian
    Romi's brain consists of a STMicroelectronics Nucleo-L476RG microcontroller board and an
    ME 405 Shoe Of Brian, an add-on board created by Dr. JR Ridgely for Mechatronics students.
    The Shoe Of Brian provides direct USB access to the flash memory on the Nucleo and allows
    for MicroPython scripts to be loaded directly onto the Nucleo without a flasher. It is also
    used for easy access to the REPL command line over serial for direct communication.
    
    For the offical Shoe of Brian documentation, see https://spluttflob.github.io/ME405-Support/index.html
    
    @subsection ss_pyb MicroPython & cotask/task_share
    All programming on Romi was created using MicroPython, a derivative of Python that is
    designed specifically for microcontrollers like the STM32. It contains built-in libraries
    for hardware-specific functions necessary to fully utilitze the STM32. It also differs
    from Python in that it is compiled, not interpreted. The reason for using pre-compiled
    Python is for speed, which makes real-time programming possible.
    
    We also took advantage of the cotask & task_share classes provided by the ME 405 Python
    code library. cotask implements cooperative multitasking with a task scheduler. Tasks are
    written as generator functions, which the cotask scheduler calls over and over based on
    priority or round-robin scheduling. task_share is a class that implements special data
    exchanges designed specifically for cotask. These classes are optimized in C and are, just
    like everything else, designed for speed. Cooperative multitasking and real-time programming
    are completely dependent on very short snippets of code, giving each other time to run when
    necessary. All of Romi's source code was written with this in mind.
    
    For the offical ME 405 Python library documentation, see https://spluttflob.github.io/ME405-Support/annotated.html

    @subsection ss_pins Romi Pin Connections
    Romi's pin connections are listed here for reference, with the STM32 pin names and wire
    colors used in the project.
    
    Drive Left:
    <ul>
        <li> Motor_Dir:  PB5 (blue)
        <li> Motor_Eff:  PB4 (green)
        <li> Enc_Ch1:    PA5 (yellow)        
        <li> Enc_Ch2:    PB3 (blue)
    </ul>
        
    Drive Right: 
    <ul>
        <li> Motor_Dir:  PA1 (blue)
        <li> Motor_Eff:  PA0 (green)
        <li> Enc_Ch1:    PB6 (yellow)
        <li> Enc_Ch2:    PB7 (blue)
    </ul>
        
    I2C Ch 1 for BNO:
    <ul>
        <li> SCL:        PB8 (blue)
        <li> SDA:        PB9 (yellow)
    </ul>
        
    Axial Line Sensors:
    <ul>
        <li> L2:         PC2 (blue)      # Far left
        <li> L1:         PC3 (green)     # Mid left
        <li> C:          PA4 (gray)      # Center, Arduino pin A2
        <li> R1:         PB0 (orange)    # Mid right, Arduino pin A3
        <li> R2:         PC1 (purple)    # Far right, Arduino pin A4
    </ul>
        
    Front Line Sensors:
    <ul>
        <li> FL2:        PA6 (blue)      # Far left         
        <li> FL1:        PA7 (green)     # Mid left
        <li> FC:         PC4 (white)     # Center
        <li> FR1:        PA3 (orange)    # Mid right 
        <li> FR2:        PA2 (purple)    # Far right
    </ul>
        
    Lidar:
    <ul>
        <li> Lidar:      PC0 (white)
    </ul>
    
    @subsection ss_pybflash PYBFLASH
    The USB connection through a Shoe Of Brian provides access to the STM32 flash memory as a
    USB storage device named 'PYBFLASH'. The source code contained in the PYBFLASH folder must
    be copied to Romi through this drive in order to run it. It runs automatically when the
    Nucleo powers up.
'''