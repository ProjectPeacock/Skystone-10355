/*
    Team:       10355 - Project Peacock
    Autonomous Program - Red Strategy #1
    Actions: Place Foundation, Place stones on Foundation, and Park
    Alliance Color: Red
    Robot Starting Position: Build zone, next to build site
    Strategy Description:
        - Grab foundation and place it in the build site
        - grab stones and place them on the foundation (as many as time will allow)
        - park under the skybridge near the center bridge
    Hardware Setup:
        - 4 mecanum wheels with encoders - encoder utilized to control program accuracy and for
             measuring distance for fwd/rev drive operation
        - Linear actuator with encoder - controls mechanism for leaning lifting mechanism forward
             and backward
        - Grabbing mechanism - controlled by continuous rotation servo - not used for this auto
        - Foundation Grabbing mechanism - controlled by two continuous rotation servos (one for
             each side of the robot)
        - 4-bar mechanism - Controlled by Rev motor with encoder. Allows for extension and
             positioning of the stone.
        - Lifting mechanism - Controlled by 1 motor with encoder. Lifts the placement system. - not used for this auto
        - Delivery mechanism - Controlled by 1 continuous rotation servo. Moves stones from intake
             mechanism to the placement mechanism. - not used for this auto
        - Intake mechanism - Controlled by 2 motors.
        - Gyro sensor located at the center of the robot - utilized to compensate for drift during
             autonomous mode operation.
        - 2 x Touch sensors - limits lift mechanism when leaning forward and backward.
    State Order:
        - FOUNDATION       // Grabs the foundation and places it in the build site
        - STONE1           // Grabs the first stone and places it on the foundation
        - STONE2           // Grabs the second stone and places it on the foundation
        - STONE3           // Grabs the third stone and places it on the foundation
        - STONE4           // Grabs the fourth stone and places it on the foundation
        - STONE5           // Grabs the fifth stone and places it on the foundation
        - PARK             // Parks under the bridge
        - HALT             // Shutdown sequence for autonomous mode
 */
package org.firstinspires.ftc.teamcode.OpModes;

/*
 * Import the classes we need to have local access to.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;
import org.firstinspires.ftc.teamcode.Libs.skystoneVuforia;

import java.util.List;

/*
 * Name the opMode and put it in the appropriate group
 */
@Autonomous(name = "Red-State (F, S, P)", group = "Red")

public class RedStateAuto extends LinearOpMode {

    /*
     * Instantiate all objects needed in this class
     */

    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;                     //Opmode
    private skystoneVuforia myVuforia = new skystoneVuforia();
    private List<Double> vuforiaTracking;   //List of Vuforia coordinates
    private List<VuforiaTrackable> myTrackables;    //List of Vuforia trackable objects
    private State state = State.FOUNDATION;    //Machine State
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {

        double timeElapsed;
        double startTime;

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
        robot.servo4Bar1.setPower(0.8);       // lower the arm to grab the stone
        robot.servo4Bar2.setPower(-0.8);
        robot.servoFoundation1.setPower(0.6);
        robot.servoFoundation2.setPower(1);
        robot.servoStone.setPower(-1);
        sleep(1000);
        robot.servoSwivel.setPower(-0.4);    // Rotate the stone into position to place
        robot.servoGrab.setPower(0.3);      // be sure the stone grabber is open
        sleep(1000);
        robot.servo4Bar1.setPower(0.2);       // lower the arm to grab the stone
        robot.servo4Bar2.setPower(-0.2);
        sleep(1000);

        /*
         * Calibrate the gyro
         */
        robot.mrGyro.calibrate();
        while (robot.mrGyro.isCalibrating()) {
            telemetry.addData("Waiting on Gyro Calibration", "");
            telemetry.update();
        }

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
        startTime = runtime.startTime();

        while (opModeIsActive()) {
            switch (state) {
                case FOUNDATION:
                    /*
                     * strafe diagonally to the foundation
                     */
                    drive.translateFromWall("front",0.3, 210, 70, 2);
                    drive.translateFromWall("front",0.1, 180, 90, 0.5);

                    /*
                     * Grab the foundation
                     */
                    robot.servoFoundation1.setPower(1);
                    robot.servoFoundation2.setPower(.6);
                    sleep(500);

                    drive.rotateGyro(0.3, 10, "right", 4);

                    /*
                     * drive towards the wall
                     */
                    drive.translateToWall(.3, 0, 50, "rear",3);

                    /*
                     * rotate the foundation towards the wall
                     */
                    drive.rotateGyro(0.3, 75, "right", 4);

                    /*
                     * drive the robot into the wall
                     */
//                    drive.translateTime(0.3,180,1);

                    /*
                     * Let go of the Foundation
                     */
                    robot.servoFoundation1.setPower(0.6);
                    robot.servoFoundation2.setPower(1);
                    robot.motorIntake1.setPower(1);
                    sleep(300);
                    robot.motorIntake1.setPower(0);

                    /*
                     * Make sure that the Lift is all the way down so it doesn't bump the skybridge
                     */
                    drive.lowerLift(0.3);

                    /*
                     * Check to see if we have time to get a stone.  If not, just Park.
                     */
                    timeElapsed = getRuntime() - startTime;
                    if (timeElapsed > 20) {
                        state = State.PARK;
                    } else {
                        state = State.STONE1;
                    }
                    break;

                case STONE1:

                    robot.motorIntake1.setPower(-1);
                    robot.motorIntake2.setPower(1);
                    robot.servoDelivery.setPower(-1);

                    /*
                     * strafe closer to the bridge
                     */
                    drive.translateFromWall("left", 0.3, 90, 55, 1);

                    /*
                     * drive towards stones
                     */
                    drive.translateTime(0.4, 0, 1.0);

                    /*
                     * turn towards the stones to pick them up
                     */
                    drive.rotateGyro(0.3, 10, "right", 0.5);

                    /*
                     * drive forward to pick up the stones
                     */
                    drive.translateTime(0.3, 0, .5);

                    /*
                     * drive back to drive under the bridge
                     */
                    drive.translateTime(0.3, 180, .5);

                    /*
                     * rotate towards the foundation
                     */
                    drive.rotateGyro(0.3, 10, "left", 0.5);

                    /*
                     * drive to the foundation
                     */
                    drive.translateTime(0.5, 180, 0.8);

                    /*
                     * eject the stone
                     */
                    robot.servoStone.setPower(1);
                    sleep(300);

                    /*
                     * Check to see if we have time to get another stone.  If not, just Park.
                     */
                    timeElapsed = getRuntime() - startTime;
                    if (timeElapsed > 20) {
                        state = State.PARK;
                    } else {
                        state = State.STONE2;
                    }
                    break;

                case STONE2:

                    /*
                     * strafe closer to the bridge
                     */
                    drive.translateFromWall("left", 0.3, 90, 55, 1);

                    /*
                     * drive towards stones
                     */
                    drive.translateTime(0.5, 0, 1.0);

                    /*
                     * turn towards the stones to pick them up
                     */
                    drive.rotateGyro(0.3, 10, "right", 0.5);

                    /*
                     * drive forward to pick up the stones
                     */
                    drive.translateTime(0.3, 0, .5);

                    /*
                     * drive back to drive under the bridge
                     */
                    drive.translateTime(0.3, 180, .5);

                    /*
                     * rotate towards the foundation
                     */
                    drive.rotateGyro(0.3, 10, "left", 0.5);

                    /*
                     * drive to the foundation
                     */
                    drive.translateTime(0.5, 180, 1.0);

                    /*
                     * eject the stone
                     */
                    robot.servoStone.setPower(1);
                    sleep(300);

                    /*
                     * Check to see if we have time to get another stone.  If not, just Park.
                     */
                    timeElapsed = getRuntime() - startTime;
                    if (timeElapsed > 20) {
                        state = State.PARK;
                    } else {
                        state = State.STONE3;
                    }
                    break;

                case STONE3:

                    /*
                     * strafe closer to the bridge
                     */
                    drive.translateFromWall("left", 0.3, 90, 55, 1);

                    /*
                     * drive towards stones
                     */
                    drive.translateTime(0.5, 0, 1.2);

                    /*
                     * turn towards the stones to pick them up
                     */
                    drive.rotateGyro(0.3, 10, "right", 0.5);

                    /*
                     * drive forward to pick up the stones
                     */
                    drive.translateTime(0.3, 0, .5);

                    /*
                     * drive back to drive under the bridge
                     */
                    drive.translateTime(0.3, 180, .5);

                    /*
                     * rotate towards the foundation
                     */
                    drive.rotateGyro(0.3, 10, "left", 0.5);

                    /*
                     * drive to the foundation
                     */
                    drive.translateTime(0.5, 180, 1.2);

                    /*
                     * eject the stone
                     */
                    robot.servoStone.setPower(1);
                    sleep(300);

                    /*
                     * Check to see if we have time to get another stone.  If not, just Park.
                     */
                    timeElapsed = getRuntime() - startTime;
                    if (timeElapsed > 20) {
                        state = State.PARK;
                    } else {
                        state = State.STONE4;
                    }
                    break;

                case STONE4:

                    /*
                     * strafe closer to the bridge
                     */
                    drive.translateFromWall("left", 0.3, 90, 55, 1);

                    /*
                     * drive towards stones
                     */
                    drive.translateTime(0.5, 0, 1.4);

                    /*
                     * turn towards the stones to pick them up
                     */
                    drive.rotateGyro(0.3, 10, "right", 0.5);

                    /*
                     * drive forward to pick up the stones
                     */
                    drive.translateTime(0.3, 0, .5);

                    /*
                     * drive back to drive under the bridge
                     */
                    drive.translateTime(0.3, 180, .5);

                    /*
                     * rotate towards the foundation
                     */
                    drive.rotateGyro(0.3, 10, "left", 0.5);

                    /*
                     * drive to the foundation
                     */
                    drive.translateTime(0.5, 180, 1.4);

                    /*
                     * eject the stone
                     */
                    robot.servoStone.setPower(1);
                    sleep(300);

                    /*
                     * Check to see if we have time to get another stone.  If not, just Park.
                     */
                    timeElapsed = getRuntime() - startTime;
                    if (timeElapsed > 20) {
                        state = State.PARK;
                    } else {
                        state = State.STONE5;
                    }
                    break;

                case STONE5:

                    /*
                     * strafe closer to the bridge
                     */
                    drive.translateFromWall("left", 0.3, 90, 55, 1);

                    /*
                     * drive towards stones
                     */
                    drive.translateTime(0.5, 0, 1.6);

                    /*
                     * turn towards the stones to pick them up
                     */
                    drive.rotateGyro(0.3, 10, "right", 0.5);

                    /*
                     * drive forward to pick up the stones
                     */
                    drive.translateTime(0.3, 0, .5);

                    /*
                     * drive back to drive under the bridge
                     */
                    drive.translateTime(0.3, 180, .5);

                    /*
                     * rotate towards the foundation
                     */
                    drive.rotateGyro(0.3, 10, "left", 0.5);

                    /*
                     * drive to the foundation
                     */
                    drive.translateTime(0.5, 180, 1.6);

                    /*
                     * eject the stone
                     */
                    robot.servoStone.setPower(1);
                    sleep(300);

                    state = State.PARK;
                    break;

                case PARK:

                    state = State.HALT;
                    break;

                case HALT:
                    drive.motorsHalt();               //Stop the motors
                    robot.motorIntake1.setPower(0);
                    robot.motorIntake2.setPower(0);
                    robot.servoDelivery.setPower(0);

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
        FOUNDATION, STONE1, STONE2, STONE3, STONE4, STONE5, PARK, HALT,
    }

}