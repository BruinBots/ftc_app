package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HardwareBruinBot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;

@Autonomous (name = "BlueBuildFull", group = "Jack")

public class BlueBuildFull extends LinearOpMode {

    HardwareBruinBot robot = new HardwareBruinBot();




    private GoldAlignDetector detector;

    private ElapsedTime     runtime = new ElapsedTime();
    //public boolean found() { return GoldAlignExample.isFound(); }
    //public boolean isAligned() { return detector.getAligned(); }
    //public double getX() { return detector.getXPosition(); }
    public void runOpMode () {




//        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");
//
//        detector = new GoldAlignDetector();
//        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
//        detector.useDefaults();
//
//        // Optional Tuning
//        detector.alignSize = 50; // Was 100 How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
//        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
//        detector.downscale = 0.4; // How much to downscale the input frames
//
//        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
//        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
//        detector.maxAreaScorer.weight = 0.005;
//
//        detector.ratioScorer.weight = 5;
//        detector.ratioScorer.perfectRatio = 1.0;
//
//        detector.enable();
        //Variable setting rotation angle;


        //Initialize hardware;
        robot.init(hardwareMap);
        // Wait for the Start button to be pushed
        while (!isStarted()) {
            // Put things to do prior to start in here
        }
        double fwdSpeed=0.3;  // Forward Speed, Normally 0.1
        double rotate = 0.2; // Rotation Speed
        double strafe = 0.5;  // Strafe Speed



        sleep(10000);


        moveBot(-1,0,0,.2);
        sleep(500);
        stopBot();

//        strafe left until 17 inches of wall.
        while (rangeSensor()>=17) {
            //find gyrostrafe
            moveBot(0,0,1,.2);
        }
        stopBot();


//        Move forward until the front touch sensor is pressed
        while (robot.frontTouchSensor.getState()) {
            moveBot(-1, 0, 0, .2);
//            sleep(2000);
        }
     stopBot();

//        Clamp latches
        robot.rightPlatformServo.setPosition(1);
        robot.leftPlatformServo.setPosition(1);
        sleep(1000);

//        Move back until the back touch sensor is pressed
        gyroHoldStopOnTouch(1,0);
        stopBot();

//        Release latches
        robot.rightPlatformServo.setPosition(0);
        robot.leftPlatformServo.setPosition(0);
        sleep(2000);

//        moveBot(-1,0,0, .2);
//        sleep(500);
//        stopBot();

//        Strafe
//        gyroStrafe(-.5,0);
        gyroHoldStrafe(0, 0, -1, 4);
        stopBot();

        //hoping to move the robot 2 seconds forwards
//        moveBot(1, 0, 0, 0.2);
//        sleep(4000);
//        stopBot();
        //This loop runs until the gold mineral is found;
        //Need to change this to "while not detected" like in the GoldAlignExample program;
//        double detectorCt = 0;  // Used to make sure the robot has settled on a heading
//        double error = 0;
        /*while(!detector.isFound())
        {
            moveBot(0, rotate, 0, 0.2);
            telemetry.addData("Is found?", detector.isFound());
            telemetry.update();
        }
        telemetry.addData("Ended loop?", detector.isFound());
        telemetry.update();*/
//        while (detector.isFound()) {
//        //boolean isAligned = false;
//        //!! This is where we need to poll the detector
//           telemetry.addData("IsAligned" , detector.getAligned()); // Is the bot aligned with the gold mineral
//           telemetry.addData("X Pos" , detector.getXPosition()); // Gold X pos.
//           telemetry.addLine("testing");
//           if(detector.getAligned())
//           {
//               fwdSpeed = 0.1;
//           }
//           else
//           {
//               fwdSpeed = 0.0;
//           }
//
//           // Rotate to align with the mineral
//           while( detectorCt <= 15) {
//               // Calculate the heading error
//               error = getXError(300,30);
//               // Command the bot to move
//               moveBot(0,error,0,0.2);
//               // Count up how long it remains aligned.  Could also use gyro heading being stable?
//                if (error == 0) {
//                    detectorCt = detectorCt + 1;
//                } else {
//                    detectorCt = 0;
//                }
//               telemetry.addData("IsAligned" , detector.getAligned()); // Is the bot aligned with the gold mineral
//               telemetry.addData("X Pos" , detector.getXPosition()); // Gold X pos.
//               telemetry.addData("error", error);
//               telemetry.addData("detectorCt",detectorCt);
//               telemetry.update();
//            }
//            // strafe into mineral
//            moveBot(0,0,.5,0.5);
//           sleep(2000);
//           stopBot();
//            //For a 0.5 second period, move the robot forward away from the lander;
//            // I don't think we need this - "sleep" takes care of this while (runtime.seconds() < 0.5)
///*            if(detector.getXPosition() < 290)
//            {
//                moveBot(0,-0.2,0,0.5);
//                //double rotate = 0.2;
//                //hwMap.leftFrontDrive.setPower(fwdSpeed + rotate); //= drive + strafe + rotate;
//                //hwMap.leftRearDrive.setPower(fwdSpeed + rotate); //= drive - strafe + rotate;
//                //hwMap.rightFrontDrive.setPower(fwdSpeed - rotate); //= drive - strafe - rotate;
//                //hwMap.rightRearDrive.setPower(fwdSpeed - rotate); //= drive + strafe - rotate;
//
//            }
//            else if(detector.getXPosition() > 310)
//            {
//                moveBot(0,0.2,0,0.5);
//                //double rotate = -0.2;
//                //hwMap.leftFrontDrive.setPower(fwdSpeed - rotate); //= drive + strafe + rotate;
//                //hwMap.leftRearDrive.setPower(fwdSpeed - rotate); //= drive - strafe + rotate;
//                //hwMap.rightFrontDrive.setPower(fwdSpeed + rotate); //= drive - strafe - rotate;
//                //hwMap.rightRearDrive.setPower(fwdSpeed + rotate); //= drive + strafe - rotate;
//            }
//            else
//            {
//                //hwMap.leftFrontDrive.setPower(fwdSpeed); //= drive + strafe + rotate;
//                //hwMap.leftRearDrive.setPower(fwdSpeed); //= drive - strafe + rotate;
//                //hwMap.rightFrontDrive.setPower(fwdSpeed); //= drive - strafe - rotate;
//                //hwMap.rightRearDrive.setPower(fwdSpeed); //= drive + strafe - rotate;
//            }
//
//            sleep(50);
//            /*hwMap.leftFrontDrive.setPower(0.00); //= drive + strafe + rotate;
//            hwMap.leftRearDrive.setPower(0.00); //= drive - strafe + rotate;
//            hwMap.rightFrontDrive.setPower(0.00); //= drive - strafe - rotate;
//            hwMap.rightRearDrive.setPower(0.00); //= drive + strafe - rotate;*/
//
//            //Keep moving and aligning so long as you can see the gold mineral
//            /*while (!detector.getAligned()) {
//
//                //Detect the rotation angle to the mineral;
//                rotate = detector.getXPosition()/480;
//
//                //The motors will be set to rotate based on the magnitude of misalignment;
//                //!! We actually need to scale the "0.1" and scale the "rotate" so that the robot moves and the rotation angle equals "rotation". Could use gyro here;
//
//                hwMap.leftFrontDrive.setPower(0.1 + rotate); //= drive + strafe + rotate;
//                hwMap.leftRearDrive.setPower(0.1 + rotate); //= drive - strafe + rotate;
//                hwMap.rightFrontDrive.setPower(0.1 - rotate); //= drive - strafe - rotate;
//                hwMap.rightRearDrive.setPower(0.1 - rotate); //= drive + strafe - rotate;
//
//            }*/
//
//            //This is where we no longer see the mineral and stop the robot
//           telemetry.update();
//        }

        //Now we move the robot forward slightly to push the mineral.
           /* hwMap.leftFrontDrive.setPower(0.1); //= drive + strafe + rotate;
            hwMap.leftRearDrive.setPower(0.1); //= drive - strafe + rotate;
            hwMap.rightFrontDrive.setPower(0.1); //= drive - strafe - rotate;
            hwMap.rightRearDrive.setPower(0.1); //= drive + strafe - rotate;
        sleep(1000);*/
//           stopBot();
//           detector.disable();
        //hwMap.leftFrontDrive.setPower(0);
        //hwMap.leftRearDrive.setPower(0);
        //hwMap.rightFrontDrive.setPower(0);
        //hwMap.rightRearDrive.setPower(0);

    }

//    public double  getXError (double centerPixel, double deadBand) {
//        // This function accepts a center pixel location (usually 300)
//        // and a dead band around that pixel and provides a
//        // proportional steering signal back
//        double responseSignal = 0;
//        double error = detector.getXPosition() - centerPixel;
//        double slope = 0.01;  // Proportional multipler
//        if ( Math.abs(error) <= deadBand ){
//            // Error is inside deadband
//            responseSignal = 0;
//        } else
//        {
//            responseSignal = slope * error;
//        }
//        return responseSignal;
//
//    }
//    public void moveBot(double drive, double rotate, double strafe, double scaleFactor)
//    {
//        // This module takes inputs, normalizes them, applies a scaleFactor, and drives the motors
//        double wheelSpeeds[] = new double[4];
//        wheelSpeeds[0] = drive + strafe - rotate;
//        wheelSpeeds[1] = drive - strafe - rotate;
//        wheelSpeeds[2] = drive - strafe + rotate;
//        wheelSpeeds[3] = drive + strafe + rotate;
//
//        double maxMagnitude = Math.abs(wheelSpeeds[0]);
//
//        // Check the WheelSpeeds to find the maximum value
//        for (int i = 1; i < wheelSpeeds.length; i++)
//        {
//            double magnitude = Math.abs(wheelSpeeds[i]);
//            if (magnitude > maxMagnitude)
//            {
//                maxMagnitude = magnitude;
//            }
//        }
//        // Normalize the WheelSpeeds against the max value
//        if (maxMagnitude > 1.0)
//        {
//            for (int i = 0; i < wheelSpeeds.length; i++)
//            {
//                wheelSpeeds[i] /= maxMagnitude;
//            }
//        }
//        // Drive the motors scaled by scaleFactor
//        hwMap.leftFrontDrive.setPower(scaleFactor * wheelSpeeds[0]);
//        hwMap.leftRearDrive.setPower(scaleFactor * wheelSpeeds[1]);
//        hwMap.rightFrontDrive.setPower(scaleFactor * wheelSpeeds[2]);
//        hwMap.rightRearDrive.setPower(scaleFactor * wheelSpeeds[3]);
//
//        //telemetry.addData("drive",drive);
//        //telemetry.addData("strafe",strafe);
//        //telemetry.addData("rotate",rotate);
//        //telemetry.addData("scaleFactor",scaleFactor);
//        //telemetry.update();
//    }
public void moveBot(double drive, double rotate, double strafe, double scaleFactor)
{
    // This module takes inputs, normalizes them to DRIVE_SPEED, and drives the motors
//        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    // How to normalize...Version 3
    //Put the raw wheel speeds into an array
    double wheelSpeeds[] = new double[4];
    wheelSpeeds[0] = drive + strafe - rotate;
    wheelSpeeds[1] = drive - strafe - rotate;
    wheelSpeeds[2] = drive - strafe + rotate;
    wheelSpeeds[3] = drive + strafe + rotate;
    // Find the magnitude of the first element in the array
    double maxMagnitude = Math.abs(wheelSpeeds[0]);
    // If any of the other wheel speeds are bigger, save that value in maxMagnitude
    for (int i = 1; i < wheelSpeeds.length; i++)
    {
        double magnitude = Math.abs(wheelSpeeds[i]);
        if (magnitude > maxMagnitude)
        {
            maxMagnitude = magnitude;
        }
    }
    // Normalize all of the magnitudes to below 1
    if (maxMagnitude > 1.0)
    {
        for (int i = 0; i < wheelSpeeds.length; i++)
        {
            wheelSpeeds[i] /= maxMagnitude;
        }
    }
    // Send the normalized values to the wheels, further scaled by the user
        robot.leftFrontDrive.setPower(scaleFactor * wheelSpeeds[0]);
        robot.leftRearDrive.setPower(scaleFactor * wheelSpeeds[1]);
        robot.rightFrontDrive.setPower(scaleFactor * wheelSpeeds[2]);
        robot.rightRearDrive.setPower(scaleFactor * wheelSpeeds[3]);





}
    public void stopBot()
    {
        // This function stops the robot
        robot.leftFrontDrive.setPower(0);
        robot.leftRearDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightRearDrive.setPower(0);
    }

    public double sonarDistance (){
        // Returns distance from the sonar sensor over an average of 4 values
        // Trying to get around noise in the sensor
        // 75 is the scaling factor between voltage and distance in INCHES
        // based on data collected on 11/17/2018
        double average;
        average = robot.sonarSensor.getVoltage();
        sleep(1);
        average = average + robot.sonarSensor.getVoltage();
        sleep(1);
        average = average + robot.sonarSensor.getVoltage();
        sleep(1);
        average = average + robot.sonarSensor.getVoltage();
        return (average*75);
    }


    public double rangeSensor (){
        double average;
        average = robot.rangeSensor.getDistance(DistanceUnit.INCH);
        sleep(1);
        average = average + robot.rangeSensor.getDistance(DistanceUnit.INCH);
        sleep(1);
        average = average + robot.rangeSensor.getDistance(DistanceUnit.INCH);
        sleep(1);
        average = average + robot.rangeSensor.getDistance(DistanceUnit.INCH);
        average = average/4;

        //the sensor is reasonably accurate before 6 inches
        //1.1 comes from having the slope being 1.1 from data collected
        if (average > 6) {
            average = average * 1.1;
        }

        return average;
    }


    public void gyroStrafe ( double speed, double heading) {
        // This function will strafe the robot at a given speed while holding a heading

        double error = getError(heading);
        double deadband = 3;
        error = getError(heading);
        if (error < 0 && Math.abs(error) > deadband) {
            // Nagative error greater than 5 degrees, left of desired heading, input positive rotation
            moveBot(0, -.25, speed, 0.6);
        } else if (error > 0 && Math.abs(error) > deadband) {
            // Positive Error greater than 5 degrees, right of desired heading, input negative rotation
            moveBot(0, 0.25, speed, 0.6);
        } else {
            // Robot is on course
            moveBot(0, 0, speed, 0.6);
        }
    }

        public double getError(double targetAngle) {

            double robotError;

            // calculate error in -179 to +180 range  (
            robotError = targetAngle - getHeading();
            while (robotError > 180) { robotError -= 360;}
            while (robotError <= -180) {robotError += 360;}
            return robotError;
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


    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.

     */

    public void gyroHoldStopOnTouch( double speed, double angle) {
        // This function drives on a specified heading until the touch sensor is pressed
        double error;
        double PCoeff = 0.1;
        // keep looping while we have time remaining.

        while (robot.backTouchSensor.getState()) {

            telemetry.addData("say", "gyrohold for the touch sensor");
            telemetry.update();
            // Update telemetry & Allow time for other processes to run.
            //error = Range.clip(getError(angle),-0.3,0.3);
            error = PCoeff * getError(angle);
            moveBot(speed, error, 0, 0.2);
        }

        //stop all motion
        stopBot();
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
            moveBot(speed, error, 0, 0.3);
        }

        //stop all motion
        stopBot();
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

    public void gyroHoldStrafe( double speed, double angle, double strafe, double holdTime) {
        // This function drives on a specified heading for a given time
        // Time is in seconds!!!!!
        ElapsedTime holdTimer = new ElapsedTime();
        double error;
        double PCoeff = 0.1;
        // keep looping while we have time remaining.
        holdTimer.reset();
        while ((!isStopRequested() && holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            //error = Range.clip(getError(angle),-0.3,0.3);
            error = PCoeff * getError(angle);
            moveBot(speed, error, strafe, 0.3);
        }

        //stop all motion
        stopBot();
    }









    }


