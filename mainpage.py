'''!@file       mainpage.py
    @author     Joseph Penrose & Paolo Navarro
    @brief      Markdown file for building documentation w/ Doxygen
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
    
    \image{inline-center} html img\track.jpg width=60%
    
    Students are responsible for any mechanical design, sensor choice and integration, and 
    programming required to complete the challenge.
    
    \image{inline-center} html img\romi.jpg width=60%
    
    @section ss_gith GitHub & Project Repository
    
    To view the full project repository, including 3D models for the added Romi hardware, visit
    the GitHub main page at: https://github.com/Jojo-Penrose/ME-405-Romi/tree/main
    
    @section ss_strat Strategy
    
    @subsection ss_line Line Following
    Romi uses an array of Pololu QTR reflectance sensors with analog output for line detection.
    The sensors produce an analog signal corresponding to the reflectance of the surface in
    view. For our purposes, the sensors produce voltages on the order of 0.2V for white paper
    and 2.3V for black lines (about 0.1V below Vdd). Romi's five line sensors are located on the
    front of the chassis, spaced 17mm apart. Romi reads each sensor and multiplies each
    value by a weight, corresponding to the side and distance from center. Sensors on Romi's
    left are negative and vice versa. Sensors further from center are weighted higher. Then,
    summing all sensor readings together (ignoring the center) produces a signed number whose sign indicates Romi's
    direction off-center and whose magnitude grows as Romi gets further off the line. Then,
    in Romi's line-following routine, the sensor value is used with a closed-loop controller
    targeting a zero value (zero being on center). The output of this controller is used to
    differentially adjust the speed of each wheel. If Romi determines the line is to the right,
    then the left wheel speeds up and the right wheel slows down. This causes Romi to veer
    off of the straight path and turn slightly right, and tries to realign the track to the
    center of the chassis. If the line is to the right, the opposite occurs. In addition,
    Romi keeps track of the *unsigned* sum of all sensors (including center) as a check. If Romi is completely off
    of the line and all sensors lose signal, then it appears to the line-follower that Romi
    is centered because the weighted sensor value is near zero. However, the unsigned sum is
    used to determine if no sensors are reading line, and if so, Romi can determine if it has
    completely lost the track or not. If Romi determines that it has lost sight of the track,
    it applies a constant reading to the line-following controller corresponding to the last
    non-zero value to determine direction.
    
    A sensor array was also attached to Romi's center along the wheel axis, but it was determined to
    be a poor location for line sensing and is not used on the final project.
    
    \image{inline-center} html img\sensors.jpg width=60%
    
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
    
    \image{inline-center} html img\dist.jpg width=60%
    
    @subsection ss_fnsh Finish Line Detection
    The track's start and finish lines are distinct from the rest of the track because they
    are the only line pattern on the track where the line abtruptly ends and a horizontal
    line crosses completely across the width of the robot. This is another use of the 
    unsigned sensor value, which is used to detect if all sensors go off at once. If they do,
    and then abruptly all go dark, Romi assumes it has crossed the finish line.
    
    @subsection ss_dead Dead Reckoning & World Position
    Romi uses a dead reckoning system that blends IMU and encoder data to constantly record
    Romi's position in world coordinates, where the initial position on startup is considered
    (X,Y) (0,0) and heading phi = 0. Romi considers the direction straight ahead as +X, and to
    the left through the center of the wheel axis as +Y. Heading phi is considered a rotation
    about the Z-axis in standard radians, where the direction of the +X-axis is 0 and the
    +Y-axis is \f$\pi / 2\f$ rad, etc. Romi's heading resets to 0 at \f$2 \pi\f$ rad when 
    crossing back over the +X-axis.
    
    The dead reckoning system works on three key assumptions:
    <ul>
        <li> For a small timestep, Romi takes perfectly circular or perfectly straight paths.
        <li> For a small timestep, Romi's wheel speeds remain constant.
        <li> Romi's tires do not slip.
    </ul>
    The first two assumptions are based on the differential-drive system. If both wheels move
    at the same speed, then Romi moves in a straight line. In a straight line maneuver, dead
    reckoning the change in position is as simple as reading the distance run by each wheel,
    using the average of the two to account for error, and using the IMU heading to determine
    the X- and Y-components of the change in world position. If the wheels have different speeds,
    then Romi's path can be represented by two arcs corresponding to the individual paths taken
    by each wheel. For a small timestep, if the wheel speeds are constant, then they will trace
    perfectly circular arcs on the ground, and geometry can be used to find the distance and
    direction between start and end points.
    
    Some definitions corresponding to arc maneuvers:
    
    \image{inline-center} html img\arcvel.jpg width=60%
    
    <ul>
        <li> &phi;, Romi's world heading direction
        <li> X, Romi's world X-position
        <li> Y, Romi's world Y-position
        <li> v<SUB>L</SUB>, the line velocity of Romi's left wheel (v = R&omega;, wheel radius and wheel angular velocity)
        <li> v<SUB>C</SUB>, the velocity of Romi's center
        <li> v<SUB>R</SUB>, the line velocity of Romi's right wheel (v = R&omega;, wheel radius and wheel angular velocity)
        <li> &sigma;<SUB>XY</SUB>, the arc center of the arc maneuver path
        <li> &omega;<SUB>XY</SUB>, the pure angular velocity of Romi's rotation about center &sigma;<SUB>XY</SUB>
        <li> r<SUB>L</SUB>, the distance from &sigma;<SUB>XY</SUB> to the floor contact at the center of Romi's left tire
        <li> r<SUB>C</SUB>, the distance from &sigma;<SUB>XY</SUB> to Romi's center
        <li> r<SUB>R</SUB>, the distance from &sigma;<SUB>XY</SUB> to the floor contact at the center of Romi's right tire
        <li> W, Romi's wheelbase width (on tire centers)
    </ul>
    
    As Romi sweeps through an arc during a small timestep, the paths taken by the two wheels and
    Romi's center can be drawn as arc lengths:
    
    \image{inline-center} html img\arcpos.jpg width=60%
    
    <ul>
        <li> &theta;, the swept change in Romi's heading during an arc maneuver
        <li> l<SUB>L</SUB>, the distance covered by Romi's left wheel
        <li> l<SUB>C</SUB>, the distance covered by Romi's center
        <li> l<SUB>R</SUB>, the distance covered by Romi's right  wheel
    </ul>
    
    The arc lengths at the wheels are derived from the encoder data of each wheel. The encoder
    measures the rotation performed by the wheel. Assuming Romi's tires do not slip and encoder
    data is accurate, the rotation in radians multiplied by the wheel radii return the distance traveled.
    Over the course of one arc maneuver, Romi's motion can be defined so long as the radius of the
    arc at Romi's center point r<SUB>C</SUB> can be determined from encoder and IMU data. As arc
    lengths, the wheel distances relate to their respective radii and the swept angle by:
    
    \f[
        r_{L} \theta = l_{L}
    \f]
    
    \f[
        r_{L} = \frac{l_{L}}{\theta}
    \f]
    
    \f[
        r_{R} = \frac{l_{R}}{\theta}
    \f]
    
    We can define Romi's center arc radius r<SUB>C</SUB> using the wheelbase width W:
    
    \f[
        r_{C} = r_{L} + \frac{W}{2}
    \f]
    
    \f[
        r_{C} = r_{L} + \frac{ r_{R} -  r_{L}}{2}
    \f]
    
    \f[
        r_{C} = \frac{1}{2}\left(r_{L} +  r_{R}\right)
    \f]
    
    Inserting the arc length relationships, r<SUB>C</SUB> is fully defined by the encoder distances
    and the swept angle measurement from the IMU.
    
    \f[
        r_{C} = \frac{1}{2}\left(\frac{l_{L}}{\theta} +  \frac{l_{R}}{\theta}\right)
    \f]
    
    \f[
        r_{C} = \frac{1}{2 \theta}\left(l_{L} +  l_{R}\right)
    \f]
    
    \f[
        r_{C} = \frac{\left(l_{L} +  l_{R}\right)}{2 \left(\phi_2 - \phi_1\right)}
    \f]
    
    Consider two points travelled by Romi in an arc maneuver, from (1) to (2). The headings and X,Y
    positions of the two points are shown, along with the swept change in heading &theta;. The
    radii drawn are Romi's center radii. Then, length &delta; is the magnitude of Romi's change in
    world coordinates from (1) to (2), and &phi;<SUB>&delta;</SUB> is the direction of Romi's change
    in world coordinates. Dead reckoning Romi's motion across the maneuver is defined by the X- and
    Y-components of the vector &delta; from (1) to (2).
    
    \image{inline-center} html img\arcdist.jpg width=60%
    
    By blowing up the triangle formed by the two center radii and &delta;, we can see in detail how
    &phi;<SUB>&delta;</SUB> is determined. 
    
    \image{inline-center} html img\righttrialt.jpg width=60%
    
    It's readily seen that the change in world heading and the swept angle are the same:
    
    \f[
        \theta = \phi_2 - \phi_1
    \f]
    
    Using the right triangle altitude theorem and alternate interior angles, it is also true that
    the small angle shaded in blue is equal to \f$\theta / 2\f$. Then:
    
    \f[
        \theta_{\delta} = \phi_1 + \frac{\theta}{2}
    \f]
    
    \f[
        \theta_{\delta} = \phi_1 + \frac{1}{2}\left(\phi_2 - \phi_1\right)
    \f]
    
    \f[
        \theta_{\delta} = \frac{1}{2}\left(\phi_1 + \phi_2\right)
    \f]
    
    The length of &delta; is simply the chord length of the full arc:
    
    \f[
        \delta = 2 r_{C} \sin{\frac{\theta}{2}}
    \f]
    
    The world coordinate changes are defined as the X- and Y-components:
    
    \f[
        \delta_X = \delta \cos{\theta_{\delta}}
    \f]
    
    \f[
        \delta_Y = \delta \sin{\theta_{\delta}}
    \f]
    
    \f[
        \delta_X = 2 r_{C} \sin{\frac{\theta}{2}} \cos{\left(\frac{1}{2}\left(\phi_1 + \phi_2\right)\right)}
    \f]
    
    \f[
        \delta_Y = 2 r_{C} \sin{\frac{\theta}{2}} \sin{\left(\frac{1}{2}\left(\phi_1 + \phi_2\right)\right)}
    \f]
    
    \f[
        \delta_X = \left(\frac{l_{L} +  l_{R}}{\phi_2 - \phi_1}\right) \sin{\left(\frac{1}{2}\left(\phi_2 - \phi_1\right)\right)} \cos{\left(\frac{1}{2}\left(\phi_1 + \phi_2\right)\right)}
    \f]
    
    \f[
        \delta_Y = \left(\frac{l_{L} +  l_{R}}{\phi_2 - \phi_1}\right) \sin{\left(\frac{1}{2}\left(\phi_2 - \phi_1\right)\right)} \sin{\left(\frac{1}{2}\left(\phi_1 + \phi_2\right)\right)}
    \f]
    
    These two closed-form equations, combined with the equation for r<SUB>C</SUB> above, fully
    define Romi's change in world position given the three key assumptions. The swept angle from the
    change in world heading takes care of the sign, and this works in all four quadrants. The derivation uses
    only directly measurable quantities on the robot, with no need for numerical integration.
    Given more time with the project, we would like to use propagated uncertainty analysis to
    determine the accuracy of this system given the inaccuracies inherent to the IMU and encoders.
    
    @section ss_comp Component Overview
    
    @subsection ss_romi Romi
    Romi is a small, differential-drive robot kit produced by Pololu, an American educational
    electronics and robotics company. Romi's chassis is circular, roughly 6" in diameter, and
    driven by two DC gearmotors on 70mm wheels. It is balanced on front and rear ball casters.
    Pololu provides the bare bones of the robot: the chassis, wheels, motors with encoders,
    and a power mainboard that interfaces with 6x AA batteries with connections.
    
    @subsection ss_hdwr Additional Hardware
    Romi uses 3D-printed parts to mount the line sensors to the front and center of the robot.
    Those parts were designed in Fusion 360 using the STEP file for the Romi chassis provided
    by Pololu. The STEP files for these parts are provided in the GitHub Repository.
    
    Additionally, a mount for the microcontroller was provided to the class by instructor Charlie
    Refvem.
    
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
        <li> C:          PA4 (gray)      # Center
        <li> R1:         PB0 (orange)    # Mid right
        <li> R2:         PC1 (purple)    # Far right
    </ul>
        
    Front Line Sensors:
    <ul>
        <li> FL2:        PA6 (blue)      # Far left         
        <li> FL1:        PA7 (green)     # Mid left
        <li> FC:         PC4 (white)     # Center
        <li> FR1:        PB1 (orange)    # Mid right 
        <li> FR2:        PC5 (purple)    # Far right
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
    
    @section ss_tech Code Techniques Overview
    
    @subsection ss_bno BNO055 Calibration
    The BNO055's accelerometer, gyroscope, and magnetometer are considered calibrated when the
    CALIB_STAT register reads 0xFF. The status for each sensor is represented in the register
    as two-bit pairs.
    
    The calibration procedure follows the methods described by Bosch Sensortech Tutorials. The
    gyroscope is calibrated by keeping the BNO still. The accelerometer is calibrated by
    holding the BNO in varying stationary orienations. The magnetometer is calibrated by moving the
    BNO in a figure-eight pattern in the air.
    
    The calibration values for each sensor are displayed in the console. The bit pairs are compared
    with a 'fully calibrated' value of 0b11. The procedure is complete once the all sensors read
    0b11. The BNO is set to CONFIG_MODE so that the OFFSET registers can be read and written as 
    spaced hexidecimal values into a calibration coefficients text file. Once the file is created,
    the BNO class detects it on startup and automatically decodes and writes the values into the
    respective OFFSET registers.
    
    On startup, after the IMU has fully powered on, Romi MasterMind requests a zero on the world
    heading phi measurement before it begins dead reckoning. Because BNO's Euler-X magnetometer
    angle (used to measure phi) is measured relative to true north, BNO records the heading at that
    zero point and uses it to offset the Euler-X reading before sending it through the phi Share.
    
    @subsection ss_lidar LIDAR Sensor Integration
    A nuance of the LIDAR sensor output is that the distance is proportional to the PWM pulse width, which can
    reach a maximum length of 2.0 ms. This will definitely cause multitasking issues, so in light
    of this, the PWM output is timed with an external interrupt such that the code is not blocked
    while waiting for a falling edge. The interrupt is called in the main.py file.
    
    @subsection ss_lines Line Sensor Integration
    Each line sensor produces its own analog value corresponding to the reflectance of the surface
    below it. One issue that our strategy runs into is that there are 'dead zones' in-between the
    sensors. When the line is straddled by two sensors on either side, even if there is a small
    amount of overlap (our sensors overlap by 1mm on either side, 17mm spacing over a 19mm line),
    the total sensor value calculated by the sum of the sensor outputs drops significantly. Without
    using a great number of sensors spaced very closely, a smoothly varying signal across the span
    of the robot cannot be attained. If the overall span of the sensors is too thin, then Romi risks
    missing dashed line sections altogether by 'threading the needle' between the gaps during a turn.
    
    To account for this, Romi heavily weights the outermost sensors much stronger than the innermost
    sensors. The weighted sensor output to the controller varies from center to outside continuously
    increasing in magnitude, but there is a dip in signal strength between the side sensors. This
    was unavoidable with the sensor spacing we used, but we carefully tuned the weights until we
    removed the 'pseudo-equilibria' within the 'dead zones,' and Romi still applies control effort
    throughout the span of the sensor readings. That is, the only place where the weighted sensor
    reading is zero (the goal of the line following controller) is when the line is centered on
    Romi's body. By combining this tune with the off-line check (which
    uses the full sensor array to determine if sight of the line was altogether lost, and applies a
    constant effort in the appropriate direction until sight is reacquired), Romi's line following
    system performs very well. It is capable at running at very high speeds, but they are avoided in
    practice to minimize the jerk and wheel slippage that comes with such erratic movement.
    
    @subsection ss_cl Closed Loop Line Following Control
    Another strength of using this continuously-variable line sensor weighting system is that the
    value can be fed into a closed-loop controller. Romi uses a closed-loop controller (Class LineCL)
    to control line following. Currently, MasterMind only requests proportional control. However, a
    full P/PI/PD/PID controller is written in LineCL, and usage only requires that the I and D gains
    be supplied at any time, either during initialization or through methods ChangeKi and ChangeKd.
    We found tuning issues with PI and PID control, and P control was very much sufficient and
    reliable for this project.
    
    @subsection ss_romimm Romi MasterMind & Completing The Term Project Challenge
    Romi MasterMind (Class RomiMM) is the heart and brain of the project. It controls what Romi does,
    how it does it, where it is, and where it's going. The states are detailed in the Class page, but
    most important to the term project are states 4 and 5: 'Do Term Project' and 'Go Home.'
    
    In 'Do Term Project,' Romi follows a scripted set of maneuvers corresponding to the rules of the
    term project challenge. They are: 
	
	<ol>
        <li> Leave the start box by travelling forward a short distance.
        <li> Line follow until reaching the obstacle by waiting for the LIDAR distance to drop below a threshold.
        <li> Navigate around the obstacle by:
            <ol>
                <li> Turn left 90 degrees and face left. (All turns in this sequence are 90 degrees.)
                <li> Travel forward past the edge of the box.
                <li> Turn right and face forward.
                <li> Travel forward past the far end of the box.
                <li> Turn right and face back towards the resumed line.
                <li> Travel forward a short distance, in case forward drift causes Romi to have touched the wrong part of the track.
                <li> Travel forward until the line sensors detect the line again.
                <li> Travel forward just enough to re-center Romi's chassis with the line.
                <li> Turn left and face forward again.
            </ol>
        <li> Resume line following until the finish line is detected using the full sum of the line sensor values (all on -> all off).
        <li> Fully enter the finish box by travelling forward a short distance.
    </ol>
    
    At this point, Romi increments its state variable and goes on to 'Go Home.' In 'Go Home,' Romi
    does a sequence of actions in a loop until it detects it is within 10mm of what it believes to
    be the Home position.
	
	<ol>
        <li> Determine the distance and direction heading between Romi's current position and Home.
        <li> Turn to face the Home direction.
        <li> Drive half of the Distance Home, watching this distance measurement during the move to stop when within 10mm of Home.
    </ol>
    
    Using this sequence, Romi's only failure modes are:
	
	<ul>
        <li> Failing to line follow during a dashed line curve if the sensors sweep through a gap too aggressively and do not find the line again.
        <li> Failing to turn and face the correct directions during obstacle avoidance, and colliding with the box.
        <li> Driving over the finish line at such a steep angle that the finish is not detected, and Romi attempts to line follow the finish box.
        <li> Error in BNO calibration on startup that causes the home heading to not zero correctly, and Romi does not understand the correct direction Home.
    </ul>
    
    Of these, line following and obstacle avoidance failures are very rare on the term project 
    challenge track. Driving over the finish line at a bad angle is rare, but does happen occasionally.
    It usually occurs when Romi reacquires the line after obstacle avoidance in such a way that
    line following is more unstable than usual when it reaches the finish, which does not happen
    often. Unfortunately, the most common error Romi makes is some kind of incorrect calibration with
    BNO. It seems to occur only when the VCP_USB connection to a host PC is unplugged, or if the
    batteries are low. It occurs infrequently on board reset, and resolves itself when reset again.
    
    @subsection ss_maneuv Maneuver Generation
    The last interesting nuance of Romi's programming is maneuver generation. For context, Romi uses
    cotask for cooperative multitasking, which requires all MainTask methods (which are the actual
    scripts ran during every program cycle) to be implemented as generator functions. Generators
    are Python functions which use 'yield' statements in place of 'returns.' What makes generators
    special is that, once they are instantiated using their inputs (much like Class object creation),
    they can be called continuously so long as there are 'yield' statements remaining in the generator.
    Every time a generator is called, it runs until its next 'yield'. By placing 'yield' statements
    inside a 'while() loop,' an infinite generator is created that can be called over and over,
    which is precisely how cotask works.
    
    Like every other task object in Romi's main code, the Romi MasterMind generator MainTask is
    designed to check for its state variable and run that particular section of its code once
    every program cycle. In order to implement maneuvers, MasterMind takes the generator concept
    one step further. The fundamental concept behind generators, cotask, and cooperative multitasking
    in general is that the code ran in each MainTask over each program cycle should run in a short
    enough timespan such that the overall performance appears seamless. So, it follows that Romi's
    maneuvers should also be broken up into very small steps.
    
    The maneuvers LineFollow, LineMove, Turn, and Face are implemented as generator methods in Class
    RomiMM. They are instantiated using their respective 'goal' variables. LineFollow takes a reference
    to where it can look up the line sensor value, LineMove takes in a target distance and speed, and
    Turn and Face take relative and absolute headings, respectively. On instantiation, the maneuver
    calculates what motions it must take. Then, entering a generator loop, it uses the Drive method to
    update the motors with the commands they need to get where Romi wants to go (i.e. drive forward,
    turn in place, etc). The maneuver generator then hits a yield statement to return to the main script.
    
    Inside the main script, Romi waits inside a yield statement loop for the maneuver to reach its
    goal by watching a maneuver flag. Every pass through the program cycle during a maneuver, MasterMind:
	
	<ol>
        <li> Runs dead reckoning to determine change in position.
        <li> Jumps into the maneuver , which checks its own goal state to evaluate whether the maneuver is complete or not.
        <li> If it is not complete, it reevaluates motor needs, updates the motors, and yields out try again on the next program cycle.
        <li> If it is complete and the goal was reached, it lowers the maneuver flag and yields out.
    </ol>
    
    As soon as the maneuver flag goes down, Romi knows that maneuver is complete. If it wants to run
    another, it simply instantiates a new maneuver generator and starts the process again. On the
    8 MHz STM32 MCU, this all happens very fast, which means Romi is capable of very complex motions.
    The maneuvers on Romi now are very simple and are really a proof-of-concept for the overall
    program structure as a robotics platform that can navigate itself in a world frame.
    
    @section ss_next Next Steps
    We consider this project to be the foundation for a very robust robotics platform that navigates
    itself in a world frame. There are many avenues of improvement, such as:
	
	<ul>
        <li> Closed-loop speed control: Greatly increase the precision of waypoint maneuvering by making
        wheel behavior highly predictable.
        <li> Motion smoothing: Reduce jerk and abrupt changes in motor commands during maneuver changes.
        This would help eliminate wheel slippage during runtime, which would improve dead reckoning.
        <li> Closed-loop waypoint maneuvering: Line following exists as a proof of concept for a
        maneuver that uses closed-loop position control that constant updates during runtime and
        active motion. The structure could be used to instead command Romi to travel to specific
        world coordinates, using defined or even optimized motion paths.
        <li> Scale and GPS: On a much larger scale, such as a robot that autonomously navigates a
        large-scale area (such as a college campus), GPS data could be used to regularly correct
        for drift errors in Romi's dead reckoning system. In general, other sensor methods could
        be used blended with this system in order to create highly accurate world navigation.
    </ul>
'''