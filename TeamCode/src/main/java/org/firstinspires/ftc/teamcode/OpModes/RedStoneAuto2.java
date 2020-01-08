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

/**
 * Import the classes we need to have local access to.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;
import org.firstinspires.ftc.teamcode.Libs.DataLogger;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;
import org.firstinspires.ftc.teamcode.Libs.skystoneVuforia;

import java.util.List;
import java.util.Locale;

/**
 * Name the opMode and put it in the appropriate group
 */
@Autonomous(name = "Red-Skystones, Foundation, Park", group = "EXPERIMENT")
//@Disabled

public class RedStoneAuto2 extends LinearOpMode {

    /**
     * Instantiate all objects needed in this class
     */

    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;                     //Opmode
    private skystoneVuforia myVuforia = new skystoneVuforia();
    private ElapsedTime runtime = new ElapsedTime();
    private double power = .6;          //Motor power for all methods
    private double heading = 90;        //Heading for all methods
    private double y = -200;            //Vuforia y stop coordinate
    private double x = -200;            //Vuforia x stop coordinate
    private double ods = 0;             //Value returned from the Optical Distance Sensor
    private List<Double> vuforiaTracking;   //List of Vuforia coordinates
    private List<VuforiaTrackable> myTrackables;    //List of Vuforia trackable objects
    private DataLogger Dl;                          //Datalogger object
    private String alliance = "blue";                //Your current alliance
    private String courseCorrect = "";
    private State state = State.LOCATE_SKYSTONE;    //Machine State


    public void runOpMode() {
        /**
         * Setup the init state of the robot.  This configures all the hardware that is defined in
         * the HardwareTestPlatform class.
         */
        robot.init(hardwareMap);

        /**
         * Instantiate the drive class
         */
        DriveMecanum drive = new DriveMecanum(robot, opMode, myVuforia, myTrackables);

        /**
         * Set the initial servo positions
         */
        robot.servoFoundation1.setPower(0.6);
        robot.servoFoundation2.setPower(1);
        robot.servoGrab.setPower(-1);
        sleep(1000);

        robot.sensorProximity.getDistance(DistanceUnit.CM);

        /**
         *  Create the DataLogger object.
         */
        createDl();

        /**
         * Calibrate the gyro
         *
         **/
        robot.mrGyro.calibrate();
        while (robot.mrGyro.isCalibrating()) {
            telemetry.addData("Waiting on Gyro Calibration", "");
            telemetry.update();
        }

        telemetry.addData(">", "System initialized and Ready");
        telemetry.addData("Proximity Sensor (CM)", robot.sensorProximity.getDistance(DistanceUnit.CM));
        telemetry.addData("Rear Facing Range (CM)", robot.wallRangeSensor.getDistance(DistanceUnit.CM));
        telemetry.update();

        /**
         * Start the opMode
         */
        waitForStart();

        while (opModeIsActive()) {
            switch (state) {

                case LOCATE_SKYSTONE:
                    /**
                     * Drive to the front wall
                     */
                    telemetry.addData("gyro value = ", robot.mrGyro.getIntegratedZValue());
                    telemetry.addData("Distance (cm)",
                            String.format(Locale.US, "%.02f", robot.wallRangeSensor.getDistance(DistanceUnit.CM)));
                    telemetry.update();

                    drive.translateTime(.3, 135, 1.75);

                    /**
                     * Raise the lift into position to be able to grab skystone
                     */
                    drive.raiseLift();

                    /**
                     * Drive close enough to the Skystones for the color sensor to detect the stones
                     */
                    drive.driveToSkystone();

                    /**
                     * Strafe across the row of stones to locate the skystone
                     */
                    drive.translateSkystone(0.2,270);

                    /**
                     * Stafe more to adjust to the center on the Skystone.
                     */
                    drive.translateTime(.2, 270, .75);

                    /**
                     * Drive forward to grab the Skystone
                     */
                    drive.translateTime(.2, 180, .5);

                    /**
                     * Grab the Skystone with the grabber.
                     */
                    robot.servoGrab.setPower(0.2);
                    sleep(1000);

                    /**
                     * Back away from the stones into a position so that we can clear the skybridge
                     */
                    drive.translateTime(.2,0,.8);

                    /**
                     * Lower the lift mechanism so that we can clear the skybridge
                     */
                    drive.lowerLift();

                    /**
                     * strafe to the Foundation
                     */
                    drive.translateTime(.4, 90, 4);

                    state = State.PLACE_FOUNDATION;
                    break;

                case PLACE_FOUNDATION:
                    /**
                     * drive forward to the foundation
                     */
                    drive.translateTime(.3, 180, 1);

                    /**
                     * Grab the foundation
                     */
                    robot.servoFoundation1.setPower(1);
                    robot.servoFoundation2.setPower(.6);
                    sleep(500);

                    /**
                     * Pull the Foundation towards the wall
                     */
                    drive.translateTime(.3,0,2.5);

                    /**
                     * rotate the foundation towards the back wall
                     */
                    robot.motorLF.setPower(0.3);
                    robot.motorLR.setPower(0.3);
                    robot.motorRF.setPower(-0.3);
                    robot.motorRR.setPower(-0.3);
                    sleep (1800);

                    /**
                     * drive the robot into the back wall; will help to align the robot
                     */
                    drive.translateTime(0.2,180,1.5);

                    /**
                     * Let go of the Foundation and the skystone (should fall onto the foundation)
                     */
                    robot.servoFoundation1.setPower(0.6);
                    robot.servoFoundation2.setPower(1);
                    robot.servoGrab.setPower(-1);
                    sleep(500);

                    /**
                     * strafe to parking position near the bridge
                     */
                    drive.translateTime(.3, 330, 2);
                    sleep (1550);

                    /**
                     * strafe out of the way
                     */
                    drive.translateTime(.2, 270, .5);

                    state = State.HALT;
                    //Exit the state
                    break;

                case HALT:
                    drive.motorsHalt();               //Stop the motors

                    //Stop the DataLogger
                    dlStop();

                    //Exit the OpMode
                    requestOpModeStop();
                    break;
            }
        }
    }

    /**
     * Setup the dataLogger
     * The dataLogger takes a set of fields defined here and sets up the file on the Android device
     * to save them to.  We then log data later throughout the class.
     */
    private void createDl() {

        Dl = new DataLogger("AutoMecanumSimpleTest" + runtime.time());
        Dl.addField("runTime");
        Dl.addField("Alliance");
        Dl.addField("State");
        Dl.addField("courseCorrect");
        Dl.addField("heading");
        Dl.addField("X");
        Dl.addField("Y");
        Dl.addField("Range Sensor Front");
        Dl.addField("Lift Down touchSensor");
        Dl.addField("Lift Up touchSensor");
        Dl.addField("Lift Forward touchSensor");
        Dl.addField("Lift Back touchSensor");
        Dl.addField("ODS");
        Dl.addField("Left Front Motor Encoder Value");
        Dl.newLine();
    }

    /**
     * Log data to the file on the phone.
     */
    private void logData() {

        Dl.addField(String.valueOf(runtime.time()));
        Dl.addField(String.valueOf(alliance));
        Dl.addField(String.valueOf(state));
        Dl.addField(String.valueOf(courseCorrect));
        Dl.addField(String.valueOf(heading));
        Dl.addField(String.valueOf(x));
        Dl.addField(String.valueOf(y));
        Dl.addField(String.valueOf(robot.rangeSensorFront));
        Dl.addField(String.valueOf(robot.touchLiftDown.getValue()));
        Dl.addField(String.valueOf(robot.touchLiftUp.getValue()));
        Dl.addField(String.valueOf(robot.touchLiftForward.getValue()));
        Dl.addField(String.valueOf(robot.touchLiftBack.getValue()));
        Dl.addField(String.valueOf(ods));
        Dl.addField(String.valueOf(robot.motorLF.getCurrentPosition()));
        Dl.newLine();
    }

    /**
     * Stop the DataLogger
     */
    private void dlStop() {
        Dl.closeDataLogger();

    }

    /**
     * Enumerate the States of the machine.
     */
    enum State {
        LOCATE_SKYSTONE, PLACE_FOUNDATION, PLACE_FIRST_STONE, GRAB_2ND_STONE,
        FIFTH_STATE, HALT, END_STATE
    }

}
