Based on the instructable at https://www.instructables.com/Rat-Operated-Vehicle/ and the code it links to at 
https://github.com/LElizabethCrawford/RatCarCode/blob/master/Rat_Car_Code.ino which was suitable for version 3 of the Elegoo Smart Car Kit.

The code posted here has been rewritten to work with the V4 kit as currently sold in 2025 which uses an Elegoo Smart Car Shield board to drive the motors.

Arduino IDE dependencies to compile and flash the code are satisfied by installing “Smartcar shield” by Dimitris Platis from the Library Manager tab.

As noted in the comments in RatCar.ino I recommend not fitting the line-tracking sensor board when building the Elegoo kit v4 and repurposing it's cable 
to access the A0, A1 and A2 pins for the control inputs as well as the +5v supply for the floor plate. 

Main user configurable parameters in the code are the MAXSPEED values which define how much PWM power should be used by the motors, and controlThreshold 
which defines how much voltage must be sensed on the A0/A1/A2 analog input pins to determine when the control wires are being touched. 

MAXSPEED values set in this code are probably sensible as a starting point depending on the weight of your drivers compartment and rat, whilst 
controlThrehold may need more tuning to determine the correct value that responds reliably when a control is touched whilst avoiding uncommanded movements.

The code posted here uses the ultrasonic sensor (but without using the servo motor to point it, it's simply left pointing directly ahead) for basic collision
avoidance. If not desired this could be easily disabled by simply setting OBSTACLE_AVOIDANCE_DISTANCE to 0 in the "USER CONFIGURABLE VALUES" section of the
code or commenting out or removing the  call to getObstacleDistance() in the main "loop()" section.
