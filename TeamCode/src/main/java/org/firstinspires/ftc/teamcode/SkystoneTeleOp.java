/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

//imports all important code that we need
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//the method that defines all of the variables in this program and translates the gamepad inputs to robot outputs
@TeleOp(name="Teleop", group="Pushbot")
public class SkystoneTeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareBruinBot robot           = new HardwareBruinBot();   // Use a Pushbot's hardware
    //double          clawOffset      = 0;                       // Servo mid position
    //final double    CLAW_SPEED      = 0.02 ;                   // sets rate to move servo

    static final double     COUNTS_PER_MOTOR_REV    = 360 ;

    @Override
    public void runOpMode() {
//        double left;
//        double right;
//        double drive;
//        double turn;
//        double max;

        float drive = 0;
        float strafe = 0;
        float rotate = 0;
        float rampUp = 0;
        float rampDown = 0;
        float clawOpen = 0;
        float clawClose = 0;
//
        boolean lowerArmMotorUp = false;
        boolean lowerArmMotorDown = false;
        boolean intakeFront = false;
        boolean intakeBack = false;
        boolean upperArmMotorOut = false;
        boolean upperArmMotorIn = false;

        boolean clawRest = true;
        boolean capstoneOut = false;
        boolean capstoneIn = false;
        boolean armOut = false;
        boolean armIn = false;
        boolean armUp = false;
        boolean armDown = false;
        boolean platformServoDown = false;
        boolean platformServoUp = false;



        ElapsedTime runtime = new ElapsedTime();

        waitForStart();



        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        //while (opModeIsActive()) {

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                // Telemetry Section ------------------------------------------------------------------
                //telemetry.addData("Lift Encoder:", robot.landerLatchLift.getCurrentPosition());
                //telemetry.addData("Arm Encoder:", robot.armRotate.getCurrentPosition());
                //telemetry.update();
                if (isStopRequested()) {
                    stop();
                    sleep(5000);
                }



                // DRIVING SECTION!!!! ----------------------------------------------------------------
                drive = gamepad2.left_stick_y;// Negative because the gamepad is weird
                strafe = -gamepad2.left_stick_x;
                rotate = gamepad2.right_stick_x;

                moveBot(drive, rotate, strafe, 0.5);



                //arm extension section
                //Dpad left moves it out and right moves it in
                armOut = gamepad2.dpad_left;
                armIn = gamepad2.dpad_right;
                if (armIn) {
                    robot.armExtend.setPower(1);
                }
                else {
                    robot.armExtend.setPower(0);
                }

                if (armOut) {
                    robot.armExtend.setPower(-1);
                }
                else {
                    robot.armExtend.setPower(0);
                }


                //arm lifting section
                //Dpad up moves it up and down moves it down
                armDown = gamepad2.dpad_up;
                armUp = gamepad2.dpad_down;
                if (armUp) {
                    robot.armLift.setPower(1);
                }
                else {
                    robot.armLift.setPower(0);
                }

                if (armDown) {
                    robot.armLift.setPower(-1);
                }
                else {
                    robot.armLift.setPower(0);
                }





                //Claw Section
                // Read the triggers and roll the Servos
                //right trigger is open and left is closed.
                //open is 1 and close is  .1
                clawOpen = gamepad2.right_trigger;
                clawClose = gamepad2.left_trigger;
                if (clawOpen > clawClose) {
                    robot.clawServo.setPosition(1);
                }
                if (clawOpen < clawClose) {
                    robot.clawServo.setPosition(.1);
                }
                //no = case yet
                //testing

                //platform servo section
                //x is down and b is up
                platformServoDown = gamepad2.x;
                platformServoUp = gamepad2.b;
                if (platformServoDown) {
                    robot.rightPlatformServo.setPower(1);
                    robot.leftPlatformServo.setPower(1);
                }
                else {
                    if (platformServoUp) {
                        robot.rightPlatformServo.setPower(-1);
                        robot.leftPlatformServo.setPower(-1);
                    }
                    else {
                        robot.rightPlatformServo.setPower(0);
                        robot.leftPlatformServo.setPower(0);
                    }
                }












                //FIXME: commented out ramp
//                //RAMP SECTION
//
//                // Read the triggers and roll the Servos
//                //one servo as of 10/20/19
//                rampUp = gamepad2.right_trigger;
//                rampDown = gamepad2.left_trigger;
//                if (rampUp > rampDown) {
////                    robot.rampServoRight.setPower(-rampUp);
//                    robot.rampServoLeft.setPower(rampUp);
//                    telemetry.addData("say", "right trigger: rampUp");
//
//                }

                //FIXME: commented out ramp
//                if (rampUp < rampDown) {
//
////                    robot.rampServoRight.setPower(rampDown);
//                    robot.rampServoLeft.setPower(-rampDown);
//                    telemetry.addData("say", "left trigger: rampDown");
//                }
//                if ( rampUp == rampDown) {
//                    telemetry.addData("say", "rampUp = rampDown");
//                }
//                telemetry.update();

//                telemetry.addData("upper arm Encoder:", robot.upperArmMotor.getCurrentPosition());
//                telemetry.addData("lower arm Encoder:", robot.lowerArmMotor.getCurrentPosition());
//                telemetry.update();
//                telemetry.addData("lower arm Encoder:", robot.upperArmMotor.getCurrentPosition());


                //FIXME: commented out lower arm
//                //LOWER ARM PLACE
//                lowerArmMotorUp = gamepad2.dpad_up;
//                lowerArmMotorDown = gamepad2.dpad_down;
////                telemetry.addData("say", "at the lower arm motor place");
////                telemetry.update();
//                if (lowerArmMotorUp) {
//                    robot.lowerArmMotor.setPower(1);
//                } else {
//                    robot.lowerArmMotor.setPower(0);
//
//                }
//
//                if (lowerArmMotorDown) {
//                    robot.lowerArmMotor.setPower(-1);
//                } else {
//                    robot.lowerArmMotor.setPower(0);
//                }

                //FIXME: commented out upper arm
//                //UPPER ARM PLACE
//                upperArmMotorOut = gamepad2.dpad_right;
//                upperArmMotorIn = gamepad2.dpad_left;
////                telemetry.addData("say", "at the upper arm motor place");
////                telemetry.update();
//
//                if (upperArmMotorOut) {
//                    robot.upperArmMotor.setPower(1);
//                } else {
//                    robot.upperArmMotor.setPower(0);
//
//                }
//
//                if (upperArmMotorIn) {
//
//                    robot.upperArmMotor.setPower(-1);
//                } else {
//                    robot.upperArmMotor.setPower(0);
//                }


                //FIXME: commented out intake
//                //INTAKE PLACE
//                intakeFront = gamepad2.left_bumper;
//                intakeBack = gamepad2.right_bumper;
//                if (intakeBack) {
//                    robot.intakeLeft.setPower(1);
//                    robot.intakeRight.setPower(-1);
//                    telemetry.addData("say", "right bumper: intakeBack");
//                }
//                else {
//                    robot.intakeLeft.setPower(0);
//                    robot.intakeRight.setPower(0);
//                }
//                if (intakeFront) {
//                    robot.intakeLeft.setPower(-1);
//                    robot.intakeRight.setPower(1);
//                    telemetry.addData("say", "left bumper: intakeFront");
//                }
//                else {
//                    robot.intakeLeft.setPower(0);
//                    robot.intakeRight.setPower(0);
//                }
//                telemetry.update();

//        if (gamepad2.a) {
//            clawOpen=true;
//            clawClose=false;
//            clawRest=false;
//        }
//
//        if (gamepad2.y) {
//            clawClose=true;
//            clawOpen=false;
//            clawRest=false;
//        }
//
//        if (!gamepad2.a && !gamepad2.y) {
//            clawRest=true;
//            clawOpen=false;
//            clawClose=false;
//        }
//
//        if (clawOpen) {
//            robot.clawMotor.setPower(1);
//        }
//        else {
//            if (clawClose) {
//                robot.clawMotor.setPower(-1);
//            }
//            else {
//                if (clawRest) {
//                    robot.clawMotor.setPower(.00000001);
//                    //because the servo is being dumb
//                }
//            }
//        }




        //FIXME: commented out capstone
//        //CAPSTONE PLACE
//        capstoneOut = gamepad2.b;
//        capstoneIn = gamepad2.x;
//
//                if (capstoneOut && !capstoneIn) {
//                    robot.capstoneServo.setPower(-1);
//                } else {
//                    if (capstoneIn && !capstoneOut) {
//                        robot.capstoneServo.setPower(1);
//                    } else {
//                        robot.capstoneServo.setPower(0);
//                    }
                } //extra?
//            }

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
//            drive = -gamepad1.left_stick_y;
//            turn  =  gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
//            left  = drive + turn;
//            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
//            max = Math.max(Math.abs(left), Math.abs(right));
//            if (max > 1.0)
//            {
//                left /= max;
//                right /= max;
//            }

            // Output the safe vales to the motor drives.
//            robot.leftDrive.setPower(left);
//            robot.rightDrive.setPower(right);

            // Use gamepad left & right Bumpers to open and close the claw
//            if (gamepad1.right_bumper)
//                clawOffset += CLAW_SPEED;
//            else if (gamepad1.left_bumper)
//                clawOffset -= CLAW_SPEED;

            // Move both servos to new position.  Assume servos are mirror image of each other.
//            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
//            robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);
//            robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);

            // Use gamepad buttons to move arm up (Y) and down (A)
//            if (gamepad1.y)
//                robot.leftArm.setPower(robot.ARM_UP_POWER);
//            else if (gamepad1.a)
//                robot.leftArm.setPower(robot.ARM_DOWN_POWER);
//            else
//                robot.leftArm.setPower(0.0);

            // Send telemetry message to signify robot running;
//            telemetry.addData("claw",  "Offset = %.2f", clawOffset);
//            telemetry.addData("left",  "%.2f", left);
//            telemetry.addData("right", "%.2f", right);
//            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
//            sleep(50);
        //}
        //driving things
    }
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
}
