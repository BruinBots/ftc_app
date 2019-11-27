package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@TeleOp(name = "Sensor Test", group = "Pushbot")
public class SensorTest extends LinearOpMode {

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
        if (isStopRequested()) {
            stop();
            sleep(5000);
        }


        // Wait for the Start button to be pushed ----------------------------START----------------------------------------------
        while (!isStarted()) {
            // Put things to do prior to start in here
            telemetry.addData("Robot Heading: ", getHeading());
            telemetry.addData("Front Touch Sensor: ", robot.frontTouchSensor.getState());
            telemetry.addData("Front Touch Sensor: ", robot.backTouchSensor.getState());
            telemetry.addData("Sonar Sensor (in): ", sonarDistance());
            telemetry.addData("Range Sensor (in): ", rangeSensor());
            telemetry.addData("Color Sensor Red: ", robot.colorSensor.red());
            telemetry.addData("Color Sensor Blue: ", robot.colorSensor.blue());
            telemetry.update();
            if (isStopRequested()) {
                stop();
                sleep(5000);
            }

        }
        while (opModeIsActive()) {
            telemetry.addData("Robot Heading: ", getHeading());
            telemetry.addData("Front Touch Sensor: ", robot.frontTouchSensor.getState());
            telemetry.addData("Front Touch Sensor: ", robot.backTouchSensor.getState());
            telemetry.addData("Sonar Sensor (in): ", sonarDistance());
            telemetry.addData("Range Sensor (in): ", rangeSensor());
            telemetry.addData("Color Sensor Red: ", robot.colorSensor.red());
            telemetry.addData("Color Sensor Blue: ", robot.colorSensor.blue());
            telemetry.update();
            if (isStopRequested()) {
                stop();
                sleep(5000);
            }
        }
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
        return (average*75);

    }
    public double rangeSensor (){
        double average;
        average = robot.rangeSensor.getDistance(DistanceUnit.INCH);
        if (average>=6) {
            average = average;
        }
        else {
            average = average*1.1;
        }
        return average;
    }
}





