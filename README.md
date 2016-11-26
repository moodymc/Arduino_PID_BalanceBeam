# Arduino_PID_BalanceBeam
Code to operate a simple ball and beam mechanism using a servo and an ultrasonic sensor.

Depending on your servo you may need to change some of the settings in the code regarding your servo max and min limits.

I needed to reverse my output from the PID controller and did this using a simple map command - You may need to comment this out depending on the action of your servo.

For deubugging puprposes it will be necessary to have the PID output sent to the serial monitor.
