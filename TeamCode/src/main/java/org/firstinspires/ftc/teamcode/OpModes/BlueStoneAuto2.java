/*
    Team:       10355 - Project Peacock
    Autonomous Program - Blue Strategy #1 - Place Foundation and park
    Alliance Color: Blue
    Robot Starting Position: Blue buidl zone, wall next to build site
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
@Autonomous(name = "Blue-Stones, Foundation, Park", group = "EXPERIMENT")

public class BlueStoneAuto2 extends LinearOpMode {

    /**
     * Instantiate all objects needed in this class
     */

    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;                     //Opmode
    double radians = 0;
    private skystoneVuforia myVuforia = new skystoneVuforia();
    private ElapsedTime runtime = new ElapsedTime();
    /**
     * Define global variables
     */
    private double mm = 500;            //Distance for translateDistance method
    private double power = .6;          //Motor power for all methods
    private double heading = 90;        //Heading for all methods
    private double y = -200;            //Vuforia y stop coordinate
    private double x = -200;            //Vuforia x stop coordinate
    private double currentZint;         //Current integrated Z value
    private double strafeTime;         //Timeout in seconds for translateTime method
    private double ods = 0;             //Value returned from the Optical Distance Sensor
    private double colorRightRed = 0;   //Value from color sensor
    private double colorRightBlue = 0;  //Value from color sensor
    private double colorLeftRed = 0;    //Value from color sensor
    private double colorLeftBlue = 0;   //Value from color sensor
    private double robotX;              // The robot's X position from VuforiaLib
    private double robotY;              // The robot's Y position from VuforiaLib
    private double robotBearing;        //Bearing to, i.e. the bearing you need to steer toward
    private List<Double> vuforiaTracking;   //List of Vuforia coordinates
    private List<VuforiaTrackable> myTrackables;    //List of Vuforia trackable objects
    private DataLogger Dl;                          //Datalogger object
    private String alliance = "blue";                //Your current alliance
    private String courseCorrect = "";
    private State state = State.LOCATESKYSTONE;    //Machine State
    private double foundationStrafe=3;  // amount of time it takes to strafe from front wall to the foundation


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
         * Set the initial position for the Lift mechanism
         */

/*        robot.motorLinear.setPower(-0.3);
        robot.motorLift.setPower(-0.3);
        while (!initialize) {
            // wait for the touch sensor to be pressed
            if (robot.touchLiftBack.isPressed()){
                robot.motorLift.setPower(0);
            }
            if (robot.touchLiftDown.isPressed()){
                robot.motorLinear.setPower(0);
            }
          }

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

        /**
         * Initialize Vuforia and retrieve the list of trackable objects.
         **/
        telemetry.addData("Waiting on Vuforia", "");
        telemetry.update();

//        myTrackables = myVuforia.vuforiaInit(myTrackables);

        telemetry.addData("Status", "Vuforia Initialized");
        telemetry.addData(">", "System initialized and Ready");
        telemetry.addData("CM", robot.sensorProximity.getDistance(DistanceUnit.CM));
        telemetry.update();

        /**
         * Start the opMode
         */
        waitForStart();

        while (opModeIsActive()) {
            switch (state) {

                case LOCATESKYSTONE:
                    /**
                     * This state is for testing the MR Gyro Sensor
                     */

                    /**
                     * Drive to the front wall
                     */

                    telemetry.addData("gyro value = ", robot.mrGyro.getIntegratedZValue());
                    telemetry.addData("Distance (cm)",
                            String.format(Locale.US, "%.02f", robot.wallRangeSensor.getDistance(DistanceUnit.CM)));
                    telemetry.update();

                    drive.translateTime(.3, 240, 1.75);

                    drive.raiseLift();

                    drive.driveToSkystone();

                    drive.translateSkystone(0.2,90);

                    /**
                     * Grab the block with the grabber.
                     */
                    drive.translateTime(.2, 90, .5);
                    robot.servoGrab.setPower(0.2);
                    sleep(1000);

                    drive.translateTime(.2,0,.8);

                    drive.lowerLift();

                    drive.translateTime(.3, 90, 2);

                    /**
                     * Strafe to the center of the Skystone
                     */
//                    drive.translateTime(0.2, .2, .3);

                    /**
                     * pickup skystone
                     */
//                    drive.translateTime(0.2, 0, 0.2);
//                    robot.servoRightGrab.setPosition(0.4);
//                    sleep(500);

                    /**
                     * Lower the lift
                     */
//                    drive.lowerLift();

                    /**
                     * Drive back to clear the sky bridge
                     */

//                    drive.translateTime(.2, 180, .4);

                    state = State.HALT;
                    break;

                case PLACE_FIRST_STONE:
                    /**
                     * This state takes the first stone to the foundation and places it.
                     */

                    /**
                     * strafeTime is calculated by calculating how much time has already been spent
                     * staffing while searching for the stone.
                     *
                     * The stafe time should be the amount of time it takes to strafe from the wall
                     * to the foundation at 30% (.3) power on the motors.
                     *
                     * Note: The last value of the calculation is the time it takes to compete all
                     * other tasks besides strafing to look for the skystone (i.e. strafe to the wall,
                     * raise the lift mechanism, move towards the stone, grab the stone, and lower
                     * the lift.
                     */
                    strafeTime = foundationStrafe - runtime.time() - 5;
                    drive.translateTime(0.3, 270, strafeTime);

                    /**
                     * drive forward to the foundation
                     */

                    drive.translateTime(0.3, 0, 200);

                    /**
                     * drop the stone; resets the position of the grabber
                     */
                    robot.servoGrab.setPower(-1);

                    drive.translateTime(0.3, 180, 200);

                    state = State.GRAB_2ND_STONE;
                    break;

                case GRAB_2ND_STONE:
                    /**
                     * locate the 2nd stone
                     */

                    /**
                     * strafe to the first stone
                     */
                    drive.translateTime(.3, 90, 1);

                    /**
                     * lower the lift
                     */
                    drive.lowerLift();

                    /**
                     * drive forward to the stones
                     */
                    drive.driveToSkystone();

                    /**
                     * locate the skystone
                     */
                    drive.translateSkystone(.2, 90);

                    /**
                     * pickup skystone
                     */
                    drive.translateTime(0.2, 0, 0.2);
                    robot.servoRightGrab.setPosition(0.4);
                    sleep(500);

                    /**
                     * back away from the stone
                     */
                    drive.translateTime(.2, 180, .2);

                    state = State.FIFTH_STATE;
                    break;

                case PLACE_FOUNDATION:
                    /**
                     * Strafe to the foundation and move it into position
                     */

                    /**
                     * strafe to the foundation
                     */
                    drive.translateTime(.3, 270, 1.5);

                    /**
                     * Drive forward into the foundation and grab it
                     */
                    drive.translateTime(.2, 0, 500);
                    /**
                     * Grab the foundation
                     */
                    robot.servoFoundation1.setPower(1);
                    robot.servoFoundation2.setPower(.6);
                    sleep(500);

                    /**
                     * drive towards the wall
                     */
                    drive.translateTime(.3,180,2.3);

                    /**
                     * rotate the foundation towards the wall
                     */
                    robot.motorLF.setPower(-.3);
                    robot.motorLR.setPower(-.3);
                    robot.motorRF.setPower(.3);
                    robot.motorRR.setPower(0.3);
                    sleep (1200);

                    /**
                     * drive the robot into the wall
                     */
                    drive.translateTime(0.2,0,1.5);

                    /**
                     * Let go of the Foundation and the stone
                     */
                    robot.servoFoundation1.setPower(0.6);
                    robot.servoFoundation2.setPower(1);
                    robot.servoGrab.setPower(-1);
                    sleep(500);

                    /**
                     * strafe to parking position
                     */
                    drive.translateTime(.3, 145, 1.5);
                    sleep (1550);

                    /**
                     * strafe out of the way
                     */
                    drive.translateTime(.2, 90, .5);

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
            /**
             * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
             * Don't change anything past this point.  Bad things could happen.
             * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
             */
        }
    }

    /**
     * Transmit telemetry.
     */
    private void telemetry() {

        telemetry.addData("Alliance", String.valueOf(alliance));
        telemetry.addData("State", String.valueOf(state));
        telemetry.addData("Heading", String.valueOf(heading));
        telemetry.addData("robotX", String.valueOf((int) robotX));
        telemetry.addData("robotY", String.valueOf((int) robotY));
        telemetry.addData("Target X", String.valueOf(x));
        telemetry.addData("Target Y", String.valueOf(y));
        telemetry.addData("robotBearing", String.valueOf((int) robotBearing));
        telemetry.addData("Current Z Int", String.valueOf(currentZint));
        telemetry.addData("touchSensor", String.valueOf(robot.touchLiftForward.getValue()));
        telemetry.addData("touchSensor", String.valueOf(robot.touchLiftBack.getValue()));
        telemetry.addData("touchSensor", String.valueOf(robot.touchLiftUp.getValue()));
        telemetry.addData("touchSensor", String.valueOf(robot.touchLiftDown.getValue()));
        telemetry.addData("ODS", String.valueOf(ods));
        telemetry.addData("Color Right Red", String.valueOf(colorRightRed));
        telemetry.addData("Color Right Blue", String.valueOf(colorRightBlue));
        telemetry.addData("Color Left Red", String.valueOf(colorLeftRed));
        telemetry.addData("Color Left Blue", String.valueOf(colorLeftBlue));
        telemetry.addData("Current Encoder Position", String.valueOf(robot.motorLF.getCurrentPosition()));
        telemetry.update();
    }


    /**
     * Catchall to get data from all sensor systems.  Updates global variables
     */
    private void getSensorData() {
        ods = robot.ods.getLightDetected();
        currentZint = robot.mrGyro.getIntegratedZValue();
        vuforiaTracking = myVuforia.getLocation(myTrackables);
        robotX = vuforiaTracking.get(0);
        robotY = vuforiaTracking.get(1);
        robotBearing = vuforiaTracking.get(2);
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
        Dl.addField("Procedure");
        Dl.addField("courseCorrect");
        Dl.addField("heading");
        Dl.addField("robotX");
        Dl.addField("robotY");
        Dl.addField("X");
        Dl.addField("Y");
        Dl.addField("robotBearing");
        Dl.addField("initZ");
        Dl.addField("currentZ");
        Dl.addField("zCorrection");
        Dl.addField("touchSensor");
        Dl.addField("ODS");
        Dl.addField("colorRightRed");
        Dl.addField("colorRightBlue");
        Dl.addField("colorLeftRed");
        Dl.addField("colorLeftBlue");
        Dl.addField("LFTargetPos");
        Dl.addField("LFMotorPos");
        Dl.addField("LF");
        Dl.addField("RF");
        Dl.addField("LR");
        Dl.addField("RR");
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
        Dl.addField(String.valueOf((int) robotX));
        Dl.addField(String.valueOf((int) robotY));
        Dl.addField(String.valueOf(x));
        Dl.addField(String.valueOf(y));
        Dl.addField(String.valueOf((int) robotBearing));
        Dl.addField(String.valueOf(currentZint));
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
        PLACE_FOUNDATION, LOCATESKYSTONE, PLACE_FIRST_STONE, GRAB_2ND_STONE,
        FIFTH_STATE, HALT, END_STATE
    }

}
