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

//all the imports
package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
//skystone hardwaremap

    //the place where all the variables are defined
public class HardwareBruinBot
{
    /* Public OpMode members. */
    public DcMotor  leftFrontDrive   = null;
    public DcMotor  leftRearDrive = null;
    public DcMotor  rightFrontDrive  = null;
    public DcMotor  rightRearDrive = null;

    public  DcMotor armExtend = null;

//    public  CRServo armLift = null;
    public  DcMotor armLift = null;
    public Servo clawServo = null;
    public CRServo leftPlatformServo;
    public CRServo rightPlatformServo;

    public BNO055IMU gyro;

//    public DcMotor  lowerArmMotor = null;
//    public DcMotor  upperArmMotor = null;

    public DcMotor intakeLeft = null;
    public DcMotor intakeRight = null;

    public CRServo    rampServoLeft = null;
//    public CRServo    rampServoRight = null;

    public  CRServo  clawMotor = null;

    public  CRServo  capstoneServo = null;


//    public DcMotor  lowerArmMotor = null;
//    public DcMotor upperArmMotor = null
//    public CRServo    rampServo = null;
//    public DigitalChannel extendArmBackStop;
//    public DigitalChannel extendArmFrontStop;

//    public ModernRoboticsI2cRangeSensor rangeSensor;
//    public ColorSensor colorSensor;
//    public AnalogInput sonarSensor;


    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    public static final double ARM_EXT_SERVO   =  0.5 ; //this must be changed later

    /* local OpMode members. */
        HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareBruinBot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontDrive = hwMap.get(DcMotor.class, "leftFrontDrive");
        leftRearDrive = hwMap.get(DcMotor.class, "leftRearDrive");
        rightFrontDrive = hwMap.get(DcMotor.class, "rightFrontDrive");
        rightRearDrive = hwMap.get(DcMotor.class, "rightRearDrive");

        armExtend = hwMap.get(DcMotor.class, "armExtend");
        armLift = hwMap.get(DcMotor.class, "armLift");
        clawServo = hwMap.get(Servo.class, "clawServo");
        //leftPlatformServo = hwMap.get(Servo.class, "leftPlatformServo");
        //rightPlatformServo = hwMap.get(Servo.class, "rightPlatformServo");

        // REV IMU Setup

        Orientation lastAngles = new Orientation();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        gyro = hwMap.get(BNO055IMU.class, "gyro");
        gyro.initialize(parameters);

        //FIXME: commented out lower and upper arm
//        lowerArmMotor = hwMap.get(DcMotor.class, "lowerArmMotor");
//        upperArmMotor = hwMap.get(DcMotor.class, "upperArmMotor");

        //FIXME: commented out intake
//        intakeLeft = hwMap.get(DcMotor.class, "intakeLeft");
//        intakeRight = hwMap.get(DcMotor.class, "intakeRight");


//only one servo right now and it's on the right side

        //FIXME: commented out ramp
//        rampServoLeft = hwMap.get(CRServo.class, "rampServoLeft");
//        rampServoRight = hwMap.get(CRServo.class, "rampServoRight");


        //FIXME:commented out claw and capstone
//        clawMotor = hwMap.get(CRServo.class, "clawMotor");
//        capstoneServo = hwMap.get(CRServo.class, "capstoneServo");



        //landerLatchLift = hwMap.get(DcMotor.class, "landerLatchLift");
        //armExtend = hwMap.get(DcMotor.class, "armExtend");
        //armRotate = hwMap.get(DcMotor.class, "armRotate");
//right trigger to go up, left trigger to go down
        //rightMineral = hwMap.get(CRServo.class, "rightMineral");

        //extendArmBackStop = hwMap.get(DigitalChannel.class, "extendArmBackStop");
        //extendArmFrontStop = hwMap.get(DigitalChannel.class, "extendArmFrontStop");
        //extendArmBackStop.setMode(DigitalChannel.Mode.INPUT);
        //extendArmFrontStop.setMode(DigitalChannel.Mode.INPUT);

        // Initialize I2C Sensors
        //colorSensor = hwMap.get(ColorSensor.class, "colorSensor");
        //rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");
        //we are using the gyro from rev expansion hub
        //FIXME figure out how to use gyro from hub
        //gyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");

        // Initialize Analog Sonar Sensor
        //sonarSensor = hwMap.get(AnalogInput.class,"sonarSensor");


        // armExt = hwMap.get(DcMotor.class, "armExt"); //arm extension
        //leftArm    = hwMap.get(DcMotor.class, "left_arm");

        //uncommented, not sure if correct.
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);



        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightRearDrive.setPower(0);
        //armExtend.setPower(0);
        //armRotate.setPower(0);
        //landerLatchLift.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //armExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        upperArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        lowerArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //armRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //armRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //landerLatchLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //landerLatchLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Set the LED on
        //colorSensor.enableLed(true);

    }
}