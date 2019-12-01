package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous (name = "Vince Autonomous", group = "Team")
public class VinceAutonomous extends LinearOpMode {

    HardwareBruinBot robot = new HardwareBruinBot();

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.4;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.2;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 5 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.1;     // Larger is more responsive, but also less stable

    static final short     STARTING_HEADING        = 315;      // Used to set the gyro offset
    static final double     fwdSpeed                = 0.5;  // Forward Speed, Normally 0.1
    static final double     rotate                  = 0.5; // Rotation Speed
    static final double     strafe                  = 0.5;  // Strafe Speed





    private ElapsedTime runtime = new ElapsedTime();
    //public boolean found() { return GoldAlignExample.isFound(); }
    //public boolean isAligned() { return detector.getAligned(); }
    //public double getX() { return detector.getXPosition(); }
    //----------------------------------runOpMode-------------------------------------------------------
    public void runOpMode () {


        // Initialize the Robot
        robot.init(hardwareMap);
        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && robot.gyro.isGyroCalibrated()) {
            sleep(50);
            idle();
            if (isStopRequested()) stop();

        }
        if (isStopRequested()) {stop(); sleep(5000);}
        robot.colorSensor.enableLed(false);

        // Wait for the Start button to be pushed ----------------------------START----------------------------------------------
        while (!isStarted()) {
            // Put things to do prior to start in here
            telemetry.addData("Robot Heading = ", getHeading());
            telemetry.update();
            if (isStopRequested()) {stop(); sleep(5000);}

        }
        while (opModeIsActive()) {
            // Move away from the wall a little bit
            gyroHold(.25, 0, .5);
            stopBot();

            // Strafe left until we're about 5 inches from the left wall (keep zero heading)
            while (!isStopRequested() && robot.rangeSensor.getDistance(DistanceUnit.INCH) > 5) {
                gyroStrafe(.3, 0);
            }
            stopBot();

            // Go forward until the touch sensor is triggered (maintain zero heading)
            while(!isStopRequested() && robot.frontTouchSensor.getState()==true){
                gyroHold(-.3,0,.1);
            }
            stopBot();

            // Latch the platform with the servos
            robot.rightPlatformServo.setPosition(1);
            robot.leftPlatformServo.setPosition((1));

            // Back up (maintian zero heading) until the rear touch sensor hits the wall
            while (!isStopRequested() && robot.backTouchSensor.getState()==true){
                gyroHold(.3,0,0.1);
            }
            stopBot();

            //  Unlatch the platform with the servos
            robot.rightPlatformServo.setPosition(-1);
            robot.leftPlatformServo.setPosition((-1));

            // Strafe right (left) until we're over the line
            //robot.colorSensor.enableLed(true);  // Turn the LED on
            while (!isStopRequested() && robot.colorSensor.red() < 50) {
                gyroStrafe(-.5, 0);
            }
            robot.colorSensor.enableLed(false); // Turn the LED off
            stopBot();
            //Victory!
/*
            //Reset the Encoder
            robot.landerLatchLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //Lower the Robot from the lander
            while (!isStopRequested() && robot.landerLatchLift.getCurrentPosition() > landingLevel) {
                robot.landerLatchLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                latchPower = -0.2;
                robot.landerLatchLift.setPower(latchPower);

            }
            robot.landerLatchLift.setPower(0);
            if (isStopRequested()) {stop(); sleep(5000);}
            //Clear the hook
            gyroHold(-0.5, 0, 0.22);
            // Move away from the lander
            gyroStrafe(0.5, 0);
            sleep(1000);
            stopBot();
            gyroSpin(0);
            if (isStopRequested()) {stop(); sleep(5000);}
            // Move closer to the wall
            gyroHold(0.4, 0, 1.5);

            // Rotate to a heading of 315R
            gyroSpin(315);


            // Move towards the wall until 7 inches away while maintaining a heading of 270
            while (!isStopRequested() && robot.rangeSensor.getDistance(DistanceUnit.INCH) > 7) {
                gyroStrafe(.5, 315);
            }
            stopBot();
            if (isStopRequested()) {stop(); sleep(5000);}
            gyroSpin(315);

            //Drive backwards maintaining 2-4 inches from the wall
            while (!isStopRequested() && sonarDistance() > 18) {
                double wsteer = wallSteer(5);
                moveBot(0.2, 0, wsteer, 0.5);
            }
            stopBot();
            if (isStopRequested()) {stop(); sleep(5000);}            //detector.disable();

            int dropTarget = 3200;  // Target for dropping totem
            int levelTarget = 400;  // Target for holding arm forward and level
            double rotatePower;
            //Reset the Encoder
            robot.armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //Rotate the arm to level position
            while (!isStopRequested() && robot.armRotate.getCurrentPosition() < levelTarget) {
                robot.armRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rotatePower = 0.25;
                robot.armRotate.setPower(rotatePower);
            }
            if (isStopRequested()) {stop(); sleep(5000);}            robot.armRotate.setPower(0);
            //Drop the Totem
            robot.rightMineral.setPower(0.7);
            sleep(1000);
            robot.rightMineral.setPower(0);


            if (isStopRequested()) {stop(); sleep(5000);}
            gyroSpin(315);

            // Let's try wall crawling for a time, then turning and homing on the crater with the distance sensor
            while (!isStopRequested() && sonarDistance() < 72) {
                double wsteer = wallSteer(7);
                moveBot(-0.25, 0, wsteer, 0.5);
            }
            stopBot();
            if (isStopRequested()) {stop(); sleep(5000);}
            gyroSpin(315);
            //Rotate the arm over the crater
            while (!isStopRequested() && robot.armRotate.getCurrentPosition() < dropTarget) {
                robot.armRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rotatePower = 0.25;
                robot.armRotate.setPower(rotatePower);
            }
            robot.armRotate.setPower(0);
            if (isStopRequested()) {stop(); sleep(5000);}

            //  Extend the arm
            while (!isStopRequested() && robot.extendArmFrontStop.getState() == false) { // As long as the front limit switch isn't pressed, move the arm forward

                robot.armExtendMotor.setPower(-0.15);
            }
            robot.armExtendMotor.setPower(0);  // Otherwise set the power to zero
            if (isStopRequested()) {stop(); sleep(5000);}

            break;
        }

 */
        }
        stop();
    }





    public void moveBot(double drive, double rotate, double strafe, double scaleFactor)
    {
        // This module takes inputs, normalizes them, applies a scaleFactor, and drives the motors
        double wheelSpeeds[] = new double[4];
        wheelSpeeds[0] = drive + strafe - rotate;
        wheelSpeeds[1] = drive - strafe - rotate;
        wheelSpeeds[2] = drive - strafe + rotate;
        wheelSpeeds[3] = drive + strafe + rotate;

        double maxMagnitude = Math.abs(wheelSpeeds[0]);

        // Check the WheelSpeeds to find the maximum value
        for (int i = 1; i < wheelSpeeds.length; i++)
        {
            double magnitude = Math.abs(wheelSpeeds[i]);
            if (magnitude > maxMagnitude)
            {
                maxMagnitude = magnitude;
            }
        }
        // Normalize the WheelSpeeds against the max value
        if (maxMagnitude > 1.0)
        {
            for (int i = 0; i < wheelSpeeds.length; i++)
            {
                wheelSpeeds[i] /= maxMagnitude;
            }
        }
        // Drive the motors scaled by scaleFactor
        robot.leftFrontDrive.setPower(scaleFactor * wheelSpeeds[0]);
        robot.leftRearDrive.setPower(scaleFactor * wheelSpeeds[1]);
        robot.rightFrontDrive.setPower(scaleFactor * wheelSpeeds[2]);
        robot.rightRearDrive.setPower(scaleFactor * wheelSpeeds[3]);

        //telemetry.addData("drive",drive);
        //telemetry.addData("strafe",strafe);
        //telemetry.addData("rotate",rotate);
        //telemetry.addData("scaleFactor",scaleFactor);
        //telemetry.update();
    }
    public void stopBot()
    {
        // This function stops the robot
        robot.leftFrontDrive.setPower(0);
        robot.leftRearDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightRearDrive.setPower(0);
    }
/*
    public double wallSteer( double distance) {
        // This function uses the side mounted ultrasonic sensor to return a
        // steering signal based on a desired distance from the wall
        // Tested to work as a strafe input to moveBot()
        // distance is in inches
        double steer = 0;
        steer = robot.rangeSensor.getDistance(DistanceUnit.INCH) - distance;
        steer = (double)Range.clip(steer, -0.25, 0.25);
        return steer;
    }
*/
    public void gyroSpin(double heading) {
        // This function spins the robot in place to a desired heading

        // Get the current heading error between actual and desired
        double error = getError(heading);
        // While we are greater than 5 degrees from desired heading (5 seems to work best)
        while (!isStopRequested() && Math.abs(error) > 5) {
            // Rotate the robot in the correct direction.
            // Don't use more than 0.3 input power or it goes too fast
            if (error < 0 && Math.abs(error) > 5) {
                moveBot(0, -0.3, 0, 0.4);
            } else {
                moveBot(0, 0.3, 0, 0.4);
            }
            //Check the error again for the next loop
            error = getError(heading);
            telemetry.addData("Heading", getHeading());
            telemetry.update();
        }
        stopBot();
    }

    public void gyroStrafe ( double speed, double heading){
        // This function will strafe the robot at a given speed while holding a heading

        double error = getError(heading);
        double deadband = 3;
        error = getError(heading);
        if (error < 0 && Math.abs(error) > deadband) {
            // Nagative error greater than 5 degrees, left of desired heading, input positive rotation
            moveBot(0, -.25, speed, 0.6);
        } else if (error > 0 && Math.abs(error) > deadband){
            // Positive Error greater than 5 degrees, right of desired heading, input negative rotation
            moveBot(0, 0.25, speed, 0.6);
        } else {
            // Robot is on course
            moveBot(0, 0, speed, 0.6);
        }

    }
    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */

    public void gyroHold( double speed, double angle, double holdTime) {
        // This function drives on a specified heading for a given time
        // Time is in seconds!!!!!
        ElapsedTime holdTimer = new ElapsedTime();
        double error;
        double PCoeff = 0.01;
        // keep looping while we have time remaining.
        holdTimer.reset();
        while ((!isStopRequested() && holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            //error = Range.clip(getError(angle),-0.3,0.3);
            error = PCoeff * getError(angle);
            moveBot(speed,error,0,0.3);
        }

        // Stop all motion;
        stopBot();
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        //double leftSpeed;
        //double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            //leftSpeed  = 0.0;
            //rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            //rightSpeed  = speed * steer;
            //leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        moveBot(speed,steer,0, speed);

        return onTarget;
    }



    /*
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getHeading();
        while (robotError > 180) { robotError -= 360;}
        while (robotError <= -180) {robotError += 360;}
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public double getHeading()
    {
        // Get the current heading.

        Orientation angles = robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double heading = -(angles.firstAngle+360)%360;

        if (heading < -180)
            heading += 360;
        else if (heading > 180)
            heading -= 360;

        return heading;

    }
    public double sonarDistance (){
        // Returns distance from the sonar sensor over an average of 4 values
        // Trying to get around noise in the sensor
        // 110 is the scaling factor between voltage and distance in INCHES
        // based on data collected on 11/17/2018
        double average;
        average = robot.sonarSensor.getVoltage();
        sleep(1);
        average = average + robot.sonarSensor.getVoltage();
        sleep(1);
        average = average + robot.sonarSensor.getVoltage();
        sleep(1);
        average = average + robot.sonarSensor.getVoltage();
        return (average*110)/4;

    }
}



