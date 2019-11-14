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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

import static java.lang.Math.abs;

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
@TeleOp(name="ArmEncoderTest", group="Pushbot")
public class ArmEncoderTest extends LinearOpMode {

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


        int currentArmLiftPosition =0;  // Used to store the currently commanded arm position
        int MAX_LIFTARM_POSITION = 70;  // ABout 70 steps from arm starting position to full extension

        ElapsedTime runtime = new ElapsedTime();

        waitForStart();

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //arm lifting section
            //Dpad up moves it up and down moves it down
            armDown = gamepad2.dpad_down;
            armUp = gamepad2.dpad_up;
            // Only change value if arm is near commanded value, prevents overdriving arm
            if (abs(currentArmLiftPosition-robot.armLift.getCurrentPosition()) < 8){
                if (armUp) {
                    currentArmLiftPosition += 10; // Add 10 to the current arm position
                    if (currentArmLiftPosition > MAX_LIFTARM_POSITION) {
                        currentArmLiftPosition = MAX_LIFTARM_POSITION; // DOn't let it go highter than Max Position
                    }
                } else {
                    if (armDown) {
                        currentArmLiftPosition -= 10; // Subtract 10 from the current arm position
                        if (currentArmLiftPosition < 0) {
                            currentArmLiftPosition = 0;  // Don't let it go lower than 0
                        }
                    }
                }
            }
            telemetry.addData("Current Commanded Pos: ",currentArmLiftPosition);
            telemetry.addData("Actual Pos: ",robot.armLift.getCurrentPosition());
            robot.armLift.setTargetPosition(currentArmLiftPosition);
            robot.armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armLift.setPower(.2);
            telemetry.update();
        }

    }

}
