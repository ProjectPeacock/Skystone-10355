/*
    Team:       10355 - Project Peacock
    Autonomous Program - Red Strategy #2 - Grab Skystone, Place Foundation, Place Skystone and park
    Alliance Color: Red
    Robot Starting Position: Red Quarry zone, wall next to depot
    Strategy Description:
        - Grab Skystone and strafe the Foundation
        - Grab Foundation and place in build site
        - Place Skystone on the Foundation
        - Park robot in under the skybridge near the center skybridge

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
        - Lifting mechanism - Controlled by 1 motor with encoder. Lifts the placement system,
            although not used for this mission.
        - Delivery mechanism - Controlled by 3 continuous rotation servos. Moves stones from intake
             mechanism to the placement mechanism. This system is not used for this mission.
        - Intake mechanism - Controlled by 1 motor. This system is not used for this mission.
        - Gyro sensor located at the center of the robot - utilized to compensate for drift during
             autonomous mode operation.
        - 2 x Touch sensors - limits lift mechanism when leaning forward and backward.
        - 1 x Rev Proximity/Color Sensor - Utilized for detecting the Skystone
        - 1 x Rev 2M Range Sensor - Located on the back of the robot to provide distance measurement
            from the wall

    State Order:
        - LOCATE_SKYSTONE       // Locates the Skystone and then strafes to the Foundation
        - PLACE_FOUNDATION      // Places the Foundation in the build site, drops the Skystone
                                // and parks
        - HALT                  // Shutdown sequence for autonomous mode
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
import java.util.Locale;

/*
 * Name the opMode and put it in the appropriate group
 */
@Autonomous(name = "Red-Stones/Found/Park 3 ", group = "STATE")
//@Disabled

public class RedStoneAdv3 extends LinearOpMode {

    /*
     * Instantiate all variables needed in this class
     */
    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;                     //Opmode
    private skystoneVuforia myVuforia = new skystoneVuforia();
    private ElapsedTime runtime = new ElapsedTime();
    private List<Double> vuforiaTracking;   //List of Vuforia coordinates
    private List<VuforiaTrackable> myTrackables;    //List of Vuforia trackable objects
    private State state = State.LOCATE_SKYSTONE1;    //Machine State

    public void runOpMode() {
        int stonePosition = 2;
        double timeElapsed;
        double strafeTime;
        double strafeTimeInit;
        double timeToSkystone = 4;
        double redColorValue = 18;
        double startTime;
        double driveDistance;
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

        robot.sensorProximity.getDistance(DistanceUnit.CM);

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

                case LOCATE_SKYSTONE1:
                    telemetry.addData("gyro value = ", robot.mrGyro.getIntegratedZValue());
                    telemetry.addData("Distance (cm)",
                            String.format(Locale.US, "%.02f", robot.wallRangeSensor.getDistance(DistanceUnit.CM)));
                    telemetry.update();

                    /*
                     * Raise the lift into position to be able to grab skystone
                     */
                    drive.raiseLift(0.5);

                    /*
                     * Strafe toward the front wall
                     */
                    drive.translateToWall(0.5, 105, 10, "right", 0.5);

                    state = State.LEFT_STONE;

                    break;

                case LEFT_STONE:
                    stonePosition = 1;
                    /*
                     * Drive forward to get to the stone
                     */
                    drive.translateFromWall(0.6, 170, 50, 0.75);
//                    drive.translateFromWall(0.1, 180, 70, 0.5);

                    /*
                     * Turn to grab the stone
                     */
                    drive.rotateGyro(0.2,30, "left", 0.5);

                    /*
                     * Grab the stone
                     */
                    if (opMode.opModeIsActive()) {   // check to make sure time has not expired
                        robot.servoGrab.setPower(0.2);
                        sleep(400);
                    }

                    /*
                     * Lower the lift
                     */
                    drive.lowerLift(0.5);

                    /*
                     * Drive to the Foundation
                     */
                    drive.translateTime(0.5, 0, .5);
                    drive.rotateGyro(0.3, 55, "left", 0.4);

                    /*
                     * Drive to the Foundation
                     */
                    drive.translateTime(0.6, 0, .5);
                    drive.translateToWall(0.6, 0, 40,  "front", 1);

                    /*
                     * Rotate towards Foundation
                     */
                    drive.rotateGyro(0.3, 80, "left", 0.4);

                    state = State.PLACE_FOUNDATION;
                    break;


                case PLACE_FOUNDATION:
                    /*
                     * drive forward to the foundation
                     */
                    drive.translateFromWall(0.5, 180, 60, 0.5);
                    drive.translateFromWall(0.1, 180, 90, 0.5);

                    /*
                     * Grab the foundation
                     */
                    if (opMode.opModeIsActive()) {   // check to make sure time has not expired
                        robot.servoFoundation1.setPower(1);
                        robot.servoFoundation2.setPower(.6);
                        sleep(500);
                    }

                    /*
                     * Pull the Foundation towards the wall
                     */
                    drive.translateToWall(.4, 0, 50, "front",30);

                    /*
                     * rotate the foundation towards the wall
                     */
                    drive.rotateGyro(0.3, 85, "right", 2);

                    /*
                     * drive the robot into the back wall; will help to align the robot
                     */
//                    drive.translateTime(0.4,180,0.5);

                    /*
                     * Let go of the Foundation and the skystone (should fall onto the foundation)
                     */

                    if (opMode.opModeIsActive()){   // check to make sure time has not expired
                        robot.servoFoundation1.setPower(0.6);
                        robot.servoFoundation2.setPower(1);
                        robot.servoGrab.setPower(-1);
                        sleep(300);
                    }
                    /*
                     * Check to see if we have time to get another stone.  If not, just Park.
                     */
                    timeElapsed = getRuntime() - startTime;
                    if (timeElapsed > 99) {
                        state = State.PARK;
                    } else if (stonePosition == 1) {
                        state = State.STONE1;
                    } else if (stonePosition == 2) {
                        state = State.STONE2;
                    } else if (stonePosition == 3) {
                        state = State.STONE3;
                    }
                    //Exit the state
                    break;

                case STONE1:
                    stonePosition = 1;
                    /*
                     * Drive forward to get to the stone
                     */
                    drive.translateFromWall(0.6, 170, 50, 0.75);
//                    drive.translateFromWall(0.1, 180, 70, 0.5);

                    /*
                     * Turn to grab the stone
                     */
                    drive.rotateGyro(0.2,30, "left", 0.5);

                    /*
                     * Grab the stone
                     */
                    if (opMode.opModeIsActive()) {   // check to make sure time has not expired
                        robot.servoGrab.setPower(0.2);
                        sleep(400);
                    }

                    /*
                     * Lower the lift
                     */
                    drive.lowerLift(0.5);

                    /*
                     * Drive to the Foundation
                     */
                    drive.translateTime(0.5, 0, .5);
                    drive.rotateGyro(0.3, 55, "left", 0.4);

                    /*
                     * Drive to the Foundation
                     */
                    drive.translateTime(0.6, 0, .5);
                    drive.translateToWall(0.6, 0, 40,  "front", 1);

                    /*
                     * Rotate towards Foundation
                     */
                    drive.rotateGyro(0.3, 80, "left", 0.4);

                    state = State.PLACE_FOUNDATION;
                    break;

                case STONE2:
                    stonePosition = 1;
                    /*
                     * Drive forward to get to the stone
                     */
                    drive.translateFromWall(0.6, 170, 50, 0.75);
//                    drive.translateFromWall(0.1, 180, 70, 0.5);

                    /*
                     * Turn to grab the stone
                     */
                    drive.rotateGyro(0.2,30, "left", 0.5);

                    /*
                     * Grab the stone
                     */
                    if (opMode.opModeIsActive()) {   // check to make sure time has not expired
                        robot.servoGrab.setPower(0.2);
                        sleep(400);
                    }

                    /*
                     * Lower the lift
                     */
                    drive.lowerLift(0.5);

                    /*
                     * Drive to the Foundation
                     */
                    drive.translateTime(0.5, 0, .5);
                    drive.rotateGyro(0.3, 55, "left", 0.4);

                    /*
                     * Drive to the Foundation
                     */
                    drive.translateTime(0.6, 0, .5);
                    drive.translateToWall(0.6, 0, 40,  "front", 1);

                    /*
                     * Rotate towards Foundation
                     */
                    drive.rotateGyro(0.3, 80, "left", 0.4);

                    state = State.PLACE_FOUNDATION;
                    break;

                case STONE3:
                    stonePosition = 1;
                    /*
                     * Drive forward to get to the stone
                     */
                    drive.translateFromWall(0.6, 170, 50, 0.75);
//                    drive.translateFromWall(0.1, 180, 70, 0.5);

                    /*
                     * Turn to grab the stone
                     */
                    drive.rotateGyro(0.2,30, "left", 0.5);

                    /*
                     * Grab the stone
                     */
                    if (opMode.opModeIsActive()) {   // check to make sure time has not expired
                        robot.servoGrab.setPower(0.2);
                        sleep(400);
                    }

                    /*
                     * Lower the lift
                     */
                    drive.lowerLift(0.5);

                    /*
                     * Drive to the Foundation
                     */
                    drive.translateTime(0.5, 0, .5);
                    drive.rotateGyro(0.3, 55, "left", 0.4);

                    /*
                     * Drive to the Foundation
                     */
                    drive.translateTime(0.6, 0, .5);
                    drive.translateToWall(0.6, 0, 40,  "front", 1);

                    /*
                     * Rotate towards Foundation
                     */
                    drive.rotateGyro(0.3, 80, "left", 0.4);

                    state = State.PLACE_FOUNDATION;
                    break;

                case PARK:
                    /*
                     * strafe to parking position near the bridge
                     */
                    drive.translateTime(1, 30, 0.6);

                    /*
                     * strafe closer to the bridge
                     */
                    drive.translateTime(.3, 90, 1);

                    state = State.HALT;         //Exit the state
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
        LOCATE_SKYSTONE1, LEFT_STONE, CENTER_STONE, RIGHT_STONE, PLACE_FOUNDATION, STONE1, STONE2, STONE3, PARK, HALT
    }

}
