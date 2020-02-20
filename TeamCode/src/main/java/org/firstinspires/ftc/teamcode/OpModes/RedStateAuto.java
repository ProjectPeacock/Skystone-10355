/*
    Team:       10355 - Project Peacock
    Autonomous Program - Red Strategy #1
    Actions: Place Foundation and Park
    Alliance Color: Red
    Robot Starting Position: Build zone, next to build site
    Strategy Description:
        - Grab foundation and place it in the build site
        - under the skybridge near the center bridge
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
@Autonomous(name = "Red-State", group = "COMP")

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
        robot.servo4Bar1.setPower(0.8);       // lower the arm to grab the stone
        robot.servo4Bar2.setPower(-0.8);
        robot.servoFoundation1.setPower(0.6);
        robot.servoFoundation2.setPower(1);
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

        while (opModeIsActive()) {
            switch (state) {
                case FOUNDATION:
                    /*
                     * strafe diagonally to the foundation
                     */
                    drive.translateFromWall(0.3, 210, 70, 2);
                    drive.translateFromWall(0.1, 180, 90, 0.5);

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

                    /**
                     * Make sure that the Lift is all the way down so it doesn't bump the skybridge
                     */
                    drive.lowerLift(0.3);

                    /*
                     * strafe to parking position
                     */
//                    drive.translateTime(.3, 330, 2.4);

                    /*
                     * strafe out of the way
                     */
  //                  drive.translateTime(0.2, 270, 0.5);

                    state = State.STONE1;
                    //Exit the state
                    break;

                case STONE1:

                    robot.motorIntake1.setPower(-1);
                    robot.motorIntake2.setPower(1);
                    robot.servoDelivery.setPower(-1);

                    drive.translateTime(0.8, 0, 1.0);

                    drive.rotateGyro(0.3, 10, "right", 0.5);

                    drive.translateTime(0.3, 0, .5);

                    drive.translateTime(0.3, 180, .5);

                    drive.rotateGyro(0.3, 10, "left", 0.5);

                    drive.translateTime(0.8, 180, 1.0);

                    state = State.HALT;
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
        FOUNDATION, STONE1, HALT,
    }

}