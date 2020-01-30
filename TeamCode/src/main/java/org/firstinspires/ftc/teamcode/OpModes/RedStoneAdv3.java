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
        robot.servoFoundation1.setPower(0.6);
        robot.servoFoundation2.setPower(1);
        robot.servoGrab.setPower(-1);
        robot.servoStone.setPower(-1);
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
                     * Drive to the front wall (strafe diagonally)
                     */
                    drive.translateTime(.6, 115, .75);

                    /*
                     * Raise the lift into position to be able to grab skystone
                     */
                    drive.raiseLift(1);

                    /*
                     * Drive close enough to the Skystone for the color sensor to detect the stones.
                     * Uses the Rev 2m Range sensor on the back of the robot to measure distance.
                     */
                    drive.translateFromWall(0.5, 180, 50, 0.5);
                    drive.translateFromWall(0.1, 180, 65, 0.5);

                    /*
                     * Strafe across the row of stones to locate the skystone. For this function,
                     * we track the time it takes to locate the Skystone so that we can make
                     * necessary adjustments to the time required to strafe to the foundation.
                     */
                    strafeTimeInit = runtime.time();    // track the starting time
                    //alphaColor should be set to the desired upper threshold for the red value
                    drive.translateSkystone(0.2,270, redColorValue, 1.5);
                    drive.translateTime(0.1,90, 0.25);
                    strafeTime = runtime.time() - strafeTimeInit;   // tracks the total time
                    stonePosition = drive.redStonePosition(strafeTime);

                    if (stonePosition == 1) {
                        /**
                         * Because the first stone is right next to the wall, we need to take a
                         * special approach to collecting the stone.  We will drive into it and
                         * rotate slightly so that we can grab it.
                         */
                        drive.translateTime(0.5, 180, .2);
                        drive.rotateGyro(0.2, 30, "right", 0.5);
                        robot.servoGrab.setPower(0.2);
                        drive.rotateGyro(0.2, 30, "left", 0.5);
                    } else {
                        /*
                         * Strafe into position to pick up the Skystone. For this side, we need to
                         * backtrack because the color sensor is on the left side of the robot.
                         */
                        drive.translateTime(.2, 90, 0.45);

                        /*
                         * Drive forward to grab the Skystone
                         */
                        drive.translateTime(.2, 180, .4);

                        if (opMode.opModeIsActive()) {   // check to make sure time has not expired
                            robot.servoGrab.setPower(0.2);
                            sleep(1000);
                        }
                    }

                    /*
                     * Back away from the stones into a position so that we can clear the skybridge
                     */
                    drive.translateTime(.5,0,.2);

                    /*
                     * Lower the lift mechanism so that we can clear the skybridge
                     */
                    drive.lowerLift(0.5);

                    /*
                     * Strafe to the Foundation.  In the middle position, the robot takes about
                     * 4 seconds to strafe to the skystone at 40% power. In general, we need to
                     * reduce the total strafe time by about 1/3 of the time it takes to locate
                     * the Skystone with the translateSkystone algorithm.
                     */
                    timeToSkystone = 1.5 - (strafeTime * 0.2);
                    drive.translateTime(1.0, 270, timeToSkystone);

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
                    drive.translateToWall(.4, 0, 40, 30);

                    /*
                     * rotate the foundation towards the wall
                     */
                    drive.rotateGyro(0.3, 85, "right", 2);

                    /*
                     * drive the robot into the back wall; will help to align the robot
                     */
                    drive.translateTime(0.4,180,0.5);

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
                    } else {
                        state = State.LOCATE_SKYSTONE2;
                    }
                    //Exit the state
                    break;

                case LOCATE_SKYSTONE2:

                    telemetry.addData("gyro value = ", robot.mrGyro.getIntegratedZValue());
                    telemetry.addData("Distance (cm)",
                            String.format(Locale.US, "%.02f", robot.wallRangeSensor.getDistance(DistanceUnit.CM)));
                    telemetry.update();
                    /*
                     * Strafe towards bridge to pick up skystone
                     */
                    drive.translateTime(1.0, 45, .4);   // diagonally towards bridge
                    drive.translateTime(1.0, 0, .5);    // straight towards stones

                    /*
                     * Drive to the closest skystone. Use the skystone position determined in the
                     * last state to decide how far to drive to get to the
                     */
                    if (stonePosition == 1) {
                        driveDistance = 160;
                        stonePosition = 2;      // increment the position so that we get the next stone over next time
                    } else if(stonePosition == 2) {
                        driveDistance = 170;
                        stonePosition = 3;      // increment the position so that we get the next stone over next time
                    } else {
                        driveDistance = 180;
                        stonePosition = 1;      // cycle to the 1st position so that we get the next stone over next time
                    }
                    // drive fast to the desired position
                    drive.translateToWall(.5, 0, driveDistance, 1);
                    sleep(100);
                    // drive back to the distance in case we overshot it
                    drive.translateFromWall(0.1, 180, driveDistance, 0.5);

                    /*
                     * Raise the lift into position to be able to grab skystone
                     * Rotate 90 degrees towards the stones
                     */
                    drive.rotateAndRaiseLift(0.3, 85, "left", 0.75);

                    /*
                     * Drive close enough to grab the Skystone.
                     * Grab the stone while driving forward.
                     */
                    drive.translateFromWall(0.4, 180, 50, 1.5);
                    if (opMode.opModeIsActive()) {   // check to make sure time has not expired
                        robot.servoGrab.setPower(0.2);
                    }
                    drive.translateFromWall(0.4, 180, 70, 0.5);

                    /*
                     * Back away from the Skystone to clear the Skybridge.
                     */
                    drive.translateToWall(0.4, 0, 50, 0.4);

                    /*
                     * Lower the lifting mechanism so that we can clear the skybridge.
                     * rotate 90 degrees to face the Foundation and place the skystone
                     */
                    drive.rotateAndLowerLift(0.3, 85, "right", 0.75);

                    /*
                     * Drive to build zone quickly - not too close to the foundation
                     */
                    drive.translateTime(1, 180, 1.0);

                    /*
                     * Approach the foundation slowly
                     */
                    drive.translateTime(0.2, 180,  0.5);

                    /*
                     * Place the stone
                     */
                    robot.servoGrab.setPower(-1);

                    /*
                     * Check to see if we have time to get another stone.  If not, just Park.
                     */
                    timeElapsed = getRuntime() - startTime;
                    if (timeElapsed > 99) {
                        state = State.PARK;
                    } else {
                        state = State.LOCATE_STONE3;
                    }
                    //Exit the state
                    break;

                case LOCATE_STONE3:

                    telemetry.addData("gyro value = ", robot.mrGyro.getIntegratedZValue());
                    telemetry.addData("Distance (cm)",
                            String.format(Locale.US, "%.02f", robot.wallRangeSensor.getDistance(DistanceUnit.CM)));
                    telemetry.update();
                    /*
                     * Strafe towards bridge to pick up skystone
                     */
                    drive.translateTime(1.0, 45, .4);   // diagonally towards bridge
                    drive.translateTime(1.0, 0, .5);    // straight towards stones

                    /*
                     * Drive to the closest stone. Use the skystone position determined in the
                     * last state to decide how far to drive to get to the stone
                     */
                    if (stonePosition == 1) {
                        driveDistance = 160;
                        stonePosition = 2;      // increment the position so that we get the next stone over next time
                    } else if(stonePosition == 2) {
                        driveDistance = 170;
                        stonePosition = 3;      // increment the position so that we get the next stone over next time
                    } else {
                        driveDistance = 180;
                        stonePosition = 1;      // cycle to the 1st position so that we get the next stone over next time
                    }

                    // drive fast to the desired position
                    drive.translateToWall(1, 0, driveDistance, 1);
                    // drive back to the distance in case we overshot it
                    drive.translateFromWall(0.3, 180, driveDistance, 0.5);

                    /*
                     * Raise the lift into position to be able to grab skystone
                     * Rotate 90 degrees towards the stones
                     */
                    drive.rotateAndRaiseLift(0.3, 85, "left", 0.75);

                    /*
                     * Drive close enough to grab the Skystone.
                     * Grab the stone while driving forward.
                     */
                    drive.translateFromWall(0.4, 180, 50, 1.5);
                    if (opMode.opModeIsActive()) {   // check to make sure time has not expired
                        robot.servoGrab.setPower(0.2);
                    }
                    drive.translateFromWall(0.4, 180, 70, 0.5);

                    /*
                     * Back away from the Skystone to clear the Skybridge.
                     */
                    drive.translateToWall(0.4, 0, 50, 0.4);

                    /*
                     * Lower the lifting mechanism so that we can clear the skybridge.
                     * rotate 90 degrees to face the Foundation and place the skystone
                     */
                    drive.rotateAndLowerLift(0.3, 85, "right", 0.75);

                    /*
                     * Drive to build zone quickly - not too close to the foundation
                     */
                    drive.translateTime(1, 180, 1.0);

                    /*
                     * Approach the foundation slowly
                     */
                    drive.translateTime(0.2, 180,  0.5);

                    /*
                     * Place the stone
                     */
                    robot.servoGrab.setPower(-1);

                    /*
                     * Check to see if we have time to get another stone.  If not, just Park.
                     */
                    timeElapsed = getRuntime() - startTime;
                    if (timeElapsed > 99) {
                        state = State.PARK;
                    } else {
                        state = State.LOCATE_STONE3;
                    }
                    //Exit the state
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
        LOCATE_SKYSTONE1, PLACE_FOUNDATION, LOCATE_SKYSTONE2, LOCATE_STONE3, PARK, HALT
    }

}
