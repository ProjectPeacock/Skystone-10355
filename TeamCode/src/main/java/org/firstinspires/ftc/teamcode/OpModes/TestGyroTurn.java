/*
    Team:       10355 - Project Peacock
    Autonomous Program - Blue Strategy #1 - Place Foundation and park
    Alliance Color: Blue
    Robot Starting Position: Blue build zone, wall next to build site
    Strategy Description:
        - Grab Foundation and place in build site
        - Park robot in under the skybridge

    Hardware Setup:
        - 4 mecanum wheels with encoders - encoder utilized to control program accuracy and for
             measuring distance for fwd/rev drive operation
        - Linear actuator with encoder - controls mechanism for leaning lifting mechanism forward
             and backward
        - Grabbing mechanism - controlled by continuous rotation servo
        - Foundation Grabbing mechanism - controlled by two continuous rotation servos (one for
             each side of the robot)
        - 4-bar mechanism - Controlled by Rev motor with encoder. Allows for extension and
             positioning of the stone.
        - Lifting mechanism - Controlled by 1 motor with encoder. Lifts the placement system.
        - Delivery mechanism - Controlled by 3 continuous rotation servos. Moves stones from intake
             mechanism to the placement mechanism.
        - Intake mechanism - Controlled by 1 motor.
        - Gyro sensor located at the center of the robot - utilized to compensate for drift during
             autonomous mode operation.
        - 2 x Touch sensors - limits lift mechanism when leaning forward and backward.
        - 1 x Motorola Camera - Utilized for Vuforia positioning of the robot on the field

    State Order:
        - FIRST STATE       // All operation is currently performed in the first state
        - HALT                          // Shutdown sequence for autonomous mode
 */

package org.firstinspires.ftc.teamcode.OpModes;

/*
 * Import the classes we need to have local access to.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;
import org.firstinspires.ftc.teamcode.Libs.skystoneVuforia;

import java.util.List;

/*
 * Name the opMode and put it in the appropriate group
 */
@Autonomous(name = "Test - Gyro Turn", group = "Test")
@Disabled

public class TestGyroTurn extends LinearOpMode {

    /*
     * Instantiate all objects needed in this class
     */

    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;                     //Opmode
    private skystoneVuforia myVuforia = new skystoneVuforia();
    private List<Double> vuforiaTracking;   //List of Vuforia coordinates
    private List<VuforiaTrackable> myTrackables;    //List of Vuforia trackable objects
    private State state = State.TEST;    //Machine State


    public void runOpMode() {
        /*
         * Setup the init state of the robot.  This configures all the hardware that is defined in
         * the HardwareTestPlatform class.
         */
        robot.init(hardwareMap);

        /*
         * Instantiate the drive class
         */
        DriveMecanum drive = new DriveMecanum(robot, opMode, myVuforia, myTrackables);

        /*
         * Set the initial servo positions
         */
        robot.servoFoundation1.setPower(0.6);
        robot.servoFoundation2.setPower(1);
        robot.servoGrab.setPower(-1);
        robot.servoStone.setPower(-1);
        sleep(1000);

        /*
         * Calibrate the gyro
         *

        robot.mrGyro.calibrate();
        while (robot.mrGyro.isCalibrating()) {
            telemetry.addData("Waiting on Gyro Calibration", "");
            telemetry.update();
        }

         */

        telemetry.addData(">", "System initialized and Ready");
        telemetry.addData("Distance Sensor CM", robot.sensorProximity.getDistance(DistanceUnit.CM));
        telemetry.addData("Color Red", robot.colorSensorRevStone.red());
        telemetry.addData("Rear Facing Range (CM) = ", robot.wallRangeSensor.getDistance(DistanceUnit.CM));
        if (robot.wallRangeSensor.getDistance(DistanceUnit.CM) > 10){
            telemetry.addData("Initialization Problem: ", "RANGE SENSOR VALUE ISSUE");
            telemetry.addData("Action: ", "CHECK POSITION OF THE ROBOT");
        }

        telemetry.update();
        /*
         * Start the opMode
         */
        waitForStart();

        while (opModeIsActive()) {
            switch (state) {

                case TEST:

                    /*
                     * Drive to the front wall
                     */
                    drive.translateToWall(1, 0, 40, "front",2);

                    /*
                     * Rotate 90 degrees to face the stones
                     */
                    drive.rotateGyro(0.2, 90, "right", 2);

                    /*
                     * Raise the lift into position to be able to grab skystone
                     */
                    drive.raiseLift(1.5);

                    /*
                     * Strafe closer to the front wall
                     */
                    drive.translateTime(0.2, 90, 0.5);

                    /*
                     * Drive Forward
                     */
                    drive.translateFromWall("front",0.4, 180, 60, 2);
                    drive.translateFromWall("front",0.05, 180, 70, 2);

                    state = State.HALT;
                    //Exit the state
                    break;

                case HALT:
                    drive.motorsHalt();               //Stop the motors

                    //Exit the OpMode
                    requestOpModeStop();
                    break;
            }
        }
    }

    /*
     * Enumerate the States of the machine.
     */
    enum State {
        TEST, HALT,
    }

}
