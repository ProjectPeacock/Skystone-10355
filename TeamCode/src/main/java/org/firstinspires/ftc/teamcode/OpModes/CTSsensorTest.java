/*
    Team:       10355 - Project Peacock
    Program:    Sensor Test

*/

package org.firstinspires.ftc.teamcode.OpModes;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;

import java.util.Locale;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "IMU Test", group = "Test")
//@Disabled
public class CTSsensorTest extends LinearOpMode {
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

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("IMU Angle", getZAngle());
            telemetry.update();
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

    /**
     * Gets the orientation of the robot using the REV IMU
     * @return the angle of the robot
     */
    private double getZAngle(){

        return (-robot.imu.getAngularOrientation().firstAngle);
    }

}