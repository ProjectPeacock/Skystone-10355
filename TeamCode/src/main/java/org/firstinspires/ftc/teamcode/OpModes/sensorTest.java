/*
    Team:       10355 - Project Peacock
    Program:    Sensor Test

*/

package org.firstinspires.ftc.teamcode.OpModes;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;

import java.util.Locale;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Sensor Test", group = "Test")
//@Disabled
public class sensorTest extends LinearOpMode {
    /*
     * Instantiate all objects needed in this class
     */
    private final static HardwareProfile robot = new HardwareProfile();

    @Override
    public void runOpMode() {

        begin();                    // initialize the robot's hardware

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Project Peacock");    //
        telemetry.update();

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        /*
         * Calibrate the gyro
         */
        robot.mrGyro.calibrate();
        while (robot.mrGyro.isCalibrating()) {
            telemetry.addData("Waiting on Gyro Calibration", "");
            telemetry.update();
        }

        telemetry.addData(">", "System initialized and Ready");
        telemetry.addData("Color Red", robot.colorSensorRevStone.red());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("COMP OpModeMecanum Active", "");    //
        telemetry.update();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            Color.RGBToHSV((int) (robot.colorSensorRevStone.red() * SCALE_FACTOR),
                    (int) (robot.colorSensorRevStone.green() * SCALE_FACTOR),
                    (int) (robot.colorSensorRevStone.blue() * SCALE_FACTOR),
                    hsvValues);

            if (hsvValues[0] > 1000) {
                telemetry.addData("Color Sensor Sees : ", "Yellow");
            } if(hsvValues[0] < 250) {
                telemetry.addData("Color Sensor Sees : ", "Black");
            } else {
                telemetry.addData("Color Sensor Sees : ", "Space");
            }
            telemetry.addData("Rear Range Sensor (CM): ", robot.wallRangeSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Gyro Sensor: ", robot.mrGyro.getIntegratedZValue());
            telemetry.addData("Rev Proximity Distance (cm) : ",
                    String.format(Locale.US, "%.02f", robot.sensorProximity.getDistance(DistanceUnit.CM)));
            telemetry.addData("Rev Color - Alpha : ", robot.colorSensorRevStone.alpha());
            telemetry.addData("Rev Color - Red : ", robot.colorSensorRevStone.red());
            telemetry.addData("Rev Color - Green : ", robot.colorSensorRevStone.green());
            telemetry.addData("Rev Color - Blue : ", robot.colorSensorRevStone.blue());
            telemetry.addData("Rev Color - Hue : ", hsvValues[0]);
            telemetry.addData("Lift Back Touch : ", robot.touchLiftBack.isPressed());
            telemetry.addData("Lift Forward Touch : ", robot.touchLiftForward.isPressed());
            telemetry.update();

            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });
            idle();
         }
    }

    private void begin() {

        /*
         * Initialize the robot's hardware.  The hardware configuration can be found in the
         * HardwareTestPlatform.java class.
         */
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

}