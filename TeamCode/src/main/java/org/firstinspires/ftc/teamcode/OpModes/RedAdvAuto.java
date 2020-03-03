/*
    Team:       10355 - Project Peacock
    Autonomous Program - Red Strategy #3 - Place Foundation, Place 2xSkystones + 1 stone and park
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
        - PLACE_FOUNDATION      // Places the Foundation in the build site, drops the Skystone
                                // and parks
        - LOCATE_SKYSTONE1      // Locates the Skystone and places it on the Foundation
        - LOCATE_SKYSTONE2      // Locates the 2nd Skystone and places it on the Foundation
        - LOCATE_STONE3         // Locates a 3rd stone and places it on the Foundation
        - PARK                  // Parks the robot under the bridge
        - HALT                  // Shutdown sequence for autonomous mode
 */

package org.firstinspires.ftc.teamcode.OpModes;

/*
 * Import the classes we need to have local access to.
 */

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Autonomous(name = "Red- Foundation, Skystones, Park", group = "STATE")
@Disabled

public class RedAdvAuto extends LinearOpMode {

    /*
     * Instantiate all variables needed in this class
     */
    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;                     //Opmode
    private skystoneVuforia myVuforia = new skystoneVuforia();
    private ElapsedTime runtime = new ElapsedTime();
    private List<Double> vuforiaTracking;   //List of Vuforia coordinates
    private List<VuforiaTrackable> myTrackables;    //List of Vuforia trackable objects
    private State state = State.PLACE_FOUNDATION;    //Machine State

    public void runOpMode() {
        double startTime;
        double timeElapsed;
        double redColorValue = 20;
        double strafeTime;
        double strafeTimeInit;
        int stonePosition=2;
        double driveDistance;
        int stoneColor;

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
        // capture the start time to insure that we have time to run the second Skystone state
        startTime = getRuntime();

        while (opModeIsActive()) {
            switch (state) {

                case PLACE_FOUNDATION:
                    telemetry.addData("gyro value = ", robot.mrGyro.getIntegratedZValue());
                    telemetry.addData("Distance (cm)",
                            String.format(Locale.US, "%.02f", robot.wallRangeSensor.getDistance(DistanceUnit.CM)));
                    telemetry.update();


                    /*
                     * strafe diagonally to the foundation
                     * This process is done in 2 steps.  The first step gets the robot close to the
                     * foundation quickly.  The second step get it in the exact position for
                     * grabbing the foundation.  This prevents the foundation and robot from bouncing
                     * off of each other.
                     */
                    /*
                     * strafe diagonally to the foundation
                     */
                    drive.translateFromWall("front",0.5, 210, 45, 0.8);
                    drive.translateFromWall("front",0.2, 180, 85, 0.5);

                    /*
                     * Grab the foundation
                     */
                    robot.servoFoundation1.setPower(1);
                    robot.servoFoundation2.setPower(.6);
                    sleep(500);

                    /*
                     * drive towards the wall
                     */
                    drive.translateToWall(0.3, 0, 25, "front",3);

                    /*
                     * rotate the foundation towards the wall
                     */
                    drive.rotateGyro(0.3, 85, "right", 4);

                    /*
                     * drive the robot into the wall
                     */
                    drive.translateTime(0.5,180,0.5);

                    /*
                     * Let go of the Foundation
                     */
                    robot.servoFoundation1.setPower(0.6);
                    robot.servoFoundation2.setPower(1);
                    sleep(500);

                    /*
                     * Make sure that the Lift is all the way down so it doesn't bump the skybridge
                     */
                    drive.lowerLift(0.3);

                    /*
                     * Strafe close to the wall to avoid the other robot
                     */
                    drive.translateTime(0.6, 270,0.5);

                    /*
                     * Strafe away from the wall slightly to insure we are not touching it.
                     * Hopefully this will help straighten the robot if we are slightly out of alignment.
                     */
                    drive.translateTime(0.1, 90,0.1);

                    state = State.LOCATE_SKYSTONE1;
                    break;

                case LOCATE_SKYSTONE1:
                    /*
                     * Drive to the front wall
                     */
                    drive.translateTime(1,0,1);
                    sleep(100);
                    drive.translateToWall(0.5, 27, 20, "front",1);
                    drive.translateFromWall("front",0.1, 180, 10, 0.5);

                    /*
                     * Rotate 90 degrees to face the stones
                     */
                    drive.rotateGyro(0.3, 82, "left", 2);

                    /*
                     * Raise the lift into position to be able to grab skystone
                     */
                    drive.raiseLift(1.5);

                    /*
                     * Strafe closer to the front wall
                     */
                    drive.translateTime(0.2, 90, 0.5);

                    /*
                     * Drive close enough to the Skystone for the color sensor to detect the stones.
                     * Uses the Rev 2m Range sensor on the back of the robot to measure distance.
                     */
                    drive.translateFromWall("front",0.3, 180, 50, 1.5);
                    drive.translateFromWall("front",0.05, 180, 70, 1.5);

                    /*
                     * Strafe across the row of stones to locate the skystone.
                     */
                    strafeTimeInit = getRuntime();
                    //alphaColor should be set to the desired upper threshold for the red value
                  //  drive.translateSkystone(0.2,270, redColorValue, 1.5);
                    strafeTime = getRuntime() - strafeTimeInit;
                    stonePosition = drive.redStonePosition(strafeTime);
                    telemetry.addData("Position of Skystone : ", stonePosition);
                    telemetry.update();

                    /*
                     * Strafe more to center on the Skystone.
                     */
//                    drive.translateTime(0.2, 270, 0.25);

                    /*
                     * Drive forward to grab the Skystone
                     */
                    drive.translateTime(.2, 180, .2);

                    /*
                     * Grab the block with the grabber.
                     */
                    if (opMode.opModeIsActive()) {   // check to make sure time has not expired
                        robot.servoGrab.setPower(0.2);
                        sleep(500);
                    }

                    /*
                     * Back away from the Skystone to clear the Skybridge.
                     */
                    drive.translateToWall(.3, 300, 10, "front",0.4);

                    /*
                     * Lower the lifting mechanism so that we can clear the skybridge.
                     */
                    drive.lowerLift(1.25);

                    /*
                     * Rotate towards the foundation to place the stone
                     */
                    drive.rotateGyro(0.2, 85, "right", 2);

                    /*
                     * Drive to build zone quickly - not too close to the foundation
                     */
                    drive.translateTime(0.7, 180, 1.4);
                    drive.translateTime(0.1, 0, 0.1); // brake

                    /*
                     * Raise the lift and the 4-bar
                     */
                    //drive.raiseLift(1.5);
                    //drive.raise4Bar(0.5);

                    /*
                     * Approach the foundation slowly
                     */
                    drive.translateFromWall("front",0.1, 180, 205, 2);

                    /*
                     * Place the stone
                     */
                   // drive.lower4Bar();
                    if (opMode.opModeIsActive()) robot.servoGrab.setPower(-1);

                    /*
                     * Lower the lift & 4Bar
                     */
                    drive.lowerLift(1);
                    drive.lower4Bar();

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
                     * Drive to the closest skystone
                     */

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
                    drive.translateToWall(.5, 0, driveDistance, "front",1);
                    sleep(100);
                    // drive back to the distance in case we overshot it
                    drive.translateFromWall("front",0.1, 180, driveDistance, 0.5);

                    /*
                     * Rotate 90 degrees towards the stones
                     * Raise the lift into position to be able to grab skystone
                     */
                    drive.rotateAndRaiseLift(0.3, 90, "left", 0.75);

                    /*
                     * Drive close enough to the Skystone for the color sensor to detect the stones.
                     * Uses the Rev 2m Range sensor on the back of the robot to measure distance.
                     */
                    drive.translateFromWall("front",0.5, 180, 50, 1.5);
                    drive.translateFromWall("front",0.05, 180, 70, 1.5);

                    /*
                     * Strafe across the row of stones to locate the skystone.
                     */
                    //alphaColor should be set to the desired upper threshold for the red value
                   // drive.translateSkystone(0.2,90, redColorValue, 1);

                    /*
                     * Strafe more to center on the Skystone.
                     */
                    drive.translateTime(.2, 90, .75);

                    /*
                     * Drive forward to grab the Skystone
                     */
                    drive.translateTime(.2, 180, .2);

                    /*
                     * Grab the block with the grabber.
                     */
                    if (opMode.opModeIsActive()) {   // check to make sure time has not expired
                        robot.servoGrab.setPower(0.2);
                        sleep(500);
                    }

                    /*
                     * Back away from the Skystone to clear the Skybridge.
                     */
                    drive.translateToWall(0.4, 0, 20, "front",0.4);

                    /*
                     * Lower the lifting mechanism so that we can clear the skybridge.
                     * rotate 90 degrees to face the Foundation and place the skystone
                     */
                    drive.rotateAndLowerLift(0.3, 90, "right", 0.75);

                    /*
                     * Drive to build zone quickly - not too close to the foundation
                     */
                    drive.translateFromWall("front",0.5, 180, 180, 2);
                    drive.translateTime(0.1, 0, 0.1); // brake

                    /*
                     * Raise the lift and the 4-bar
                     */
                    drive.raiseLift(1.5);
                    drive.raise4Bar(0.5);

                    /*
                     * Approach the foundation slowly
                     */
                    drive.translateFromWall("front",0.1, 180, 205, 2);

                    /*
                     * Place the stone
                     */
                    drive.lower4Bar();
                    robot.servoGrab.setPower(-1);

                    /*
                     * Lower the lift & 4Bar
                     */
                    drive.lowerLift(1.5);
                    drive.lower4Bar();

                    /*
                     * Check to see if we have time to get another stone.  If not, just Park.
                     */
                    timeElapsed = getRuntime() - startTime;
                    if (timeElapsed > 24) {
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
                    drive.translateToWall(.5, 0, driveDistance, "front",1);
                    sleep(100);
                    // drive back to the distance in case we overshot it
                    drive.translateFromWall("front",0.1, 180, driveDistance, 0.5);

                    /*
                     * Rotate 90 degrees towards the stones
                     */
                    drive.rotateGyro(0.3, 90, "left", 0.75);

                    /*
                     * Raise the lift into position to be able to grab skystone
                     */
                    drive.raiseLift(1.5);

                    /*
                     * Drive close enough to the Skystone for the color sensor to detect the stones.
                     * Uses the Rev 2m Range sensor on the back of the robot to measure distance.
                     */
                    drive.translateFromWall("front",0.4, 180, 50, 1.5);
                    drive.translateFromWall("front",0.05, 180, 70, 1.5);

                    /*
                     * Strafe across the row of stones to locate the skystone.
                     */
                    //alphaColor should be set to the desired upper threshold for the red value
                   // drive.translateSkystone(0.2,90, redColorValue, 1);

                    /*
                     * Strafe more to center on the Skystone.
                     */
                    drive.translateTime(.2, 90, .75);

                    /*
                     * Drive forward to grab the Skystone
                     */
                    drive.translateTime(.2, 180, .2);

                    /*
                     * Grab the block with the grabber.
                     */
                    if (opMode.opModeIsActive()) {   // check to make sure time has not expired
                        robot.servoGrab.setPower(0.2);
                        sleep(500);
                    }

                    /*
                     * Back away from the Skystone to clear the Skybridge.
                     */
                    drive.translateToWall(0.4, 0, 20, "front",0.4);

                    /*
                     * Lower the lifting mechanism so that we can clear the skybridge.
                     */
                    drive.lowerLift(1.25);

                    /*
                     * Get closer to the wall before turning.
                     */
                    drive.translateToWall(0.1, 0, 20, "front",0.1);

                    /*
                     * rotate 90 degrees to face the Founation and place the skystone
                     */
                    drive.rotateGyro(0.3, 90, "right", 0.75);

                    /*
                     * Drive to build zone quickly - not too close to the foundation
                     */
                    drive.translateFromWall("front",0.5, 180, 180, 2);
                    drive.translateTime(0.1, 0, 0.1); // brake

                    /*
                     * Raise the lift and the 4-bar
                     */
                    drive.raiseLift(1.5);
                    drive.raise4Bar(0.5);

                    /*
                     * Approach the foundation slowly
                     */
                    drive.translateFromWall("front",0.1, 180, 205, 2);

                    /*
                     * Place the stone
                     */
                    drive.lower4Bar();
                    robot.servoGrab.setPower(-1);

                    /*
                     * Lower the lift & 4Bar
                     */
                    drive.lowerLift(1.5);
                    drive.lower4Bar();

                    state = State.PARK;
                    //Exit the state
                    break;

                case PARK:
                    /*
                     * Drive to parking position under the bridge
                     */
                    drive.translateToWall(0.5, 0, 170, "front",1.1);
                    drive.translateTime(0.1, 180, 0.1); // brake

                    /*
                     * strafe closer to the wall
                     */
                    drive.translateTime(.4, 90, 0.3);

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
        PLACE_FOUNDATION, LOCATE_SKYSTONE1, LOCATE_SKYSTONE2, LOCATE_STONE3, PARK, HALT
    }

}
