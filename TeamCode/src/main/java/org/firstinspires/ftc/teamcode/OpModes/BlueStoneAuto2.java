/*
    Team:       10355 - Project Peacock
    Autonomous Program - Blue Strategy #2 - Grab Skystone, Place Foundation, Place Skystone and park
    Alliance Color: Blue
    Robot Starting Position: Blue Quarry zone, wall next to depot
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
@Autonomous(name = "Blue-Skystones, Foundation, Park", group = "Blue")
//@Disabled

public class BlueStoneAuto2 extends LinearOpMode {

    /**
     * Instantiate all variables needed in this class
     */
    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;                     //Opmode
    private skystoneVuforia myVuforia = new skystoneVuforia();
    private ElapsedTime runtime = new ElapsedTime();
    private List<Double> vuforiaTracking;   //List of Vuforia coordinates
    private List<VuforiaTrackable> myTrackables;    //List of Vuforia trackable objects
    private State state = State.LOCATESKYSTONE;    //Machine State
    private double strafeTime = 0;
    private double strafeTimeInit = 0;
    private double timeToSkystone = 4;
    private double redColorValue = 18;

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

                case LOCATESKYSTONE:
                    telemetry.addData("gyro value = ", robot.mrGyro.getIntegratedZValue());
                    telemetry.addData("Distance (cm)",
                            String.format(Locale.US, "%.02f", robot.wallRangeSensor.getDistance(DistanceUnit.CM)));
                    telemetry.addData("Color Red", robot.colorSensorRevStone.red());
                    telemetry.update();

                    /*
                     * Drive to the front wall (strafe diagonally)
                     */
                    drive.translateTime(.3, 240, 1.6);

                    /*
                     * Raise the lift into position to be able to grab skystone
                     */
                    drive.raiseLift(2);

                    /*
                     * Drive close enough to the Skystone for the color sensor to detect the stones.
                     * Uses the Rev 2m Range sensor on the back of the robot to measure distance.
                     */
                    drive.translateFromWall("front",0.1, 180, 62, 1.5);

                    /*
                     * Strafe across the row of stones to locate the skystone. For this function,
                     * we track the time it takes to locate the Skystone so that we can make
                     * necessary adjustments to the time required to strafe to the foundation.
                     */
                    strafeTimeInit = runtime.time();
                    //alphaColor should be set to the desired upper threshold for the red value
                   /* drive.translateSkystone(0.2,90, redColorValue, 1.5);
                    strafeTime = runtime.time() - strafeTimeInit;

                    /*
                     * Stafe more to center on the Skystone.
                     */
                    drive.translateTime(.2, 90, .75);

                    /*
                     * Drive forward to grab the Skystone
                     */
                    drive.translateTime(.2, 180, .3);

                    /*
                     * Grab the block with the grabber.
                     */
                    if (opMode.opModeIsActive()) {   // check to make sure time has not expired
                        robot.servoGrab.setPower(0.2);
                        sleep(1000);
                    }

                    /*
                     * Back away from the Skystone to clear the Skybridge.
                     */
                    drive.translateTime(.3,0,.4);

                    /*
                     * Lower the lifting mechanism so that we can clear the skybridge.
                     */
                    drive.lowerLift(1.25);

                    /*
                     * Strafe to the Foundation.  In the middle position, the robot takes about
                     * 4 seconds to strafe to the skystone at 40% power. In general, we need to
                     * reduce the total stafe time by about 1/3 of the time it takes to locate
                     * the Skystone with the translateSkystone algorithm.
                     */
                    timeToSkystone = 4 - (strafeTime * 0.3);
                    drive.translateTime(.4, 90, timeToSkystone);

                    state = State.PLACE_FOUNDATION;
                    break;

                case PLACE_FOUNDATION:
                    /*
                     * drive forward to the foundation
                     */
//                    drive.translateFromWall(0.3, 180, 90, 1);
                    drive.translateFromWall("front",0.3, 180, 70, 2);
                    drive.translateFromWall("front",0.1, 180, 90, 0.5);

                    /*
                     * Grab the foundation
                     */
                    if (opMode.opModeIsActive()) {   // check to make sure time has not expired
                        robot.servoFoundation1.setPower(1);
                        robot.servoFoundation2.setPower(.6);
                        sleep(500);
                    }

                    /*
                     * Pull the foundation towards the wall.
                     */
                    drive.translateToWall(.3, 0, 20, "front",30);

                    /*
                     * rotate the foundation towards the wall
                     */
                    drive.rotateGyro(0.3, 90, "left", 3);

                    /*
                     * drive the robot into the wall
                     */
                    drive.translateTime(0.2,180,2.0);

                    /*
                     * Let go of the Foundation and the stone
                     */
                    if (opMode.opModeIsActive()) {   // check to make sure time has not expired
                        robot.servoFoundation1.setPower(0.6);
                        robot.servoFoundation2.setPower(1);
                        robot.servoGrab.setPower(-1);
                        sleep(500);
                    }
                    sleep(4000);

                    state = State.PARK_BRIDGE;      //Exit the state
                    break;

                case PARK_BRIDGE:
                    /*
                     * strafe to parking position near the bridge
                     */
                    drive.translateTime(.3, 350, 2);

                    /*
                     * strafe closer to the bridge
                     */
                    drive.translateTime(.4, 270, 1);

                    state = State.HALT;         //Exit the state
                    break;

                case PARK_WALL:
                    /*
                     * strafe closer to the wall
                     */
                    drive.translateTime(.4, 90, 0.7);

                    /*
                     * Drive to parking position under the bridge
                     */
                    drive.translateTime(.5, 0, 1.3);

                    /*
                     * strafe closer to the wall
                     */
                    drive.translateTime(.4, 90, 0.5);

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
        PLACE_FOUNDATION, LOCATESKYSTONE, PARK_BRIDGE, PARK_WALL, HALT,
    }
}
