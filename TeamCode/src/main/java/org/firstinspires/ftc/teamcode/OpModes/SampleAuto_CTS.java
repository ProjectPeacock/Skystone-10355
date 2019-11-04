/*
    Team:       10355 - Project Peacock
    Autonomous Program - Red Strategy #1
    Alliance Color: Red
    Robot Starting Position: Red zone, wall near ramp
    Strategy Description:
        - Press correct button on beacon near ramp
        - Press correct button on beacon furthest from ramp
        - Park a wheel on the red ramp

    Hardware Setup:
        - 4 mecanum wheels with encoder on LF wheel - encoder utilized for measuring distance for fwd/rev drive operation
        - Arm Motor with encoder - controls mechanism for dumping particles into ramp
        - Gyro sensor located at the center of the robot - utilized to compensate for drift
        - 1 x Color sensor (colorSensorLeft)- utilized to identify beacon color
        - 1 x Touch sensor - utilized to identify when robot touches wall with the front of the robot
        - 1 x Optical Distance Sensor (ODS) - utilized to locate the white lines on the floor
        - 1 x Motorola Camera - Utilized for Vuforia positioning of the robot on the field

    State Order:
        - FIRST STATE       // moves from the wall to the first beacon closest to the ramp
        - SECOND STATE               // Identifies which button to press, right or left
        - THIRD STATE                  // presses the correct button - also validates that the button is pressed
                                        // and attempts to press over again until the button is pressed
        - FOURTH STATE      // moves from the wall to the second beacon on the right of the field
        - FIFTH STATE               // Identifies which button to press, right or left
        - SIXTH STATE                  // presses the correct button - also validates that the button is pressed
                                        // and attempts to press over again until the button is pressed
        - END_GAME                      // identifies the last actions before the end of autonomous mode
        - HALT                          // Shutdown sequence for autonomous mode

 */
package org.firstinspires.ftc.teamcode.OpModes;

/**
 * Import the classes we need to have local access to.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Libs.DataLogger;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;
import org.firstinspires.ftc.teamcode.Libs.skystoneVuforia;
import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile_CTS;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * Name the opMode and put it in the appropriate group
 */
@Autonomous(name = "Sample Autonomous", group = "COMP")
//@Disabled
public class SampleAuto_CTS extends LinearOpMode {

    /**
     * Instantiate all objects needed in this class
     */

    private final static HardwareProfile_CTS robot = new HardwareProfile_CTS();
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
    private double changeSpeed = 0;     //Rotation speed for motor speed calculations
    private double initZ;               //The initial reading from integrated Z
    private double currentZint;         //Current integrated Z value
    private double zCorrection;         //Course correction in degrees
    private double timeOut = 5;         //Timeout in seconds for translateTime method
    private double timeOutTime;         //Calculated time to stop
    private String procedure;           //Name of the method that is running
    private double odsThreshold = .3;   //Threshold at which the ODS sensor acquires the whie line
    private double ods = 0;             //Value returned from the Optical Distance Sensor
    private double colorRightRed = 0;   //Value from color sensor
    private double colorRightBlue = 0;  //Value from color sensor
    private double colorLeftRed = 0;    //Value from color sensor
    private double colorLeftBlue = 0;   //Value from color sensor
    private double robotX;              // The robot's X position from VuforiaLib
    private double robotY;              // The robot's Y position from VuforiaLib
    private double robotBearing;        //Bearing to, i.e. the bearing you need to steer toward
    private double robotRoll;
    private double visibleTarget;
    private double LF, RF, LR, RR;      //Motor power setting
    private double myCurrentMotorPosition;  //Current encoder position
    private double myTargetPosition;        //Target encoder position
    private boolean leftAlign = false;      //
    private List<Double> vuforiaTracking;   //List of Vuforia coordinates
    private List<VuforiaTrackable> myTrackables;    //List of Vuforia trackable objects
    private DataLogger Dl;                          //Datalogger object
    private double motorCorrectCoefficient = .05;    //Amount to divide the zInt drift by
    private String button;                          //Which button to push
    private String alliance = "red";                //Your current alliance
    private String courseCorrect = "";
    private State state = State.VUFORIA_TEST;    //Machine State
    private boolean initialize = false;


   /**
     * Setting up Vuforia to operate with the autonomous opMode.
     */
    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;


    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AQnFmuP/////AAABmQCm3/d2XkmSr01g6YE3lFSJSgwnTpe+36Ubl1lNvf5SfnVBBt2kW6Jl/wXN//IX3CfcSOldD2PTFY56tUPu0R45yCVtA3+y33VKlzKXMR1nFbYjOvD6BiG2fmDIx8ViGKvq0tr1NZQo8XpeTVL8N79dxMSHzUHBoehIzrtniGKoeaYcr4H6oGAxOXp0GLebanWq61B6VWxKp4etuwzS9OX86R+PMVAXHBTJLWpOm2WeTIeCopZ46wfpzVZDeI6BEXHN84QzFGM8g4lmTxwBizXxUE08tlOjTl+V/+EkkDsMHys+x9f/hyXCetITnAmRiiueOFzYhx5XBItX9msyLLU/TinrMIPICW7U5IAU8kdh";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch            = 25.4f;
    private static final float mmTargetHeight       = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ               = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ              = 6.42f * mmPerInch;
    private static final float bridgeY              = 23 * mmPerInch;
    private static final float bridgeX              = 5.18f * mmPerInch;
    private static final float bridgeRotY           = 59;                                 // Units are degrees
    private static final float bridgeRotZ           = 180;

    // Constants for perimeter targets
    private static final float halfField            = 72 * mmPerInch;
    private static final float quadField            = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation               = null;
    private VuforiaLocalizer vuforia                = null;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
//    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;


    public void runOpMode() {
        /**
         * Setup the init state of the robot.  This configures all the hardware that is defined in
         * the HardwareTestPlatform class.
         */
        robot.init(hardwareMap);

        /**
         * Initialize Vuforia and retrieve the list of trackable objects.
         */
        telemetry.addData("Waiting on Vuforia", "");
        telemetry.update();

        VuforiaTrackables skystoneTrackables = vuforia.loadTrackablesFromAsset("FTC_2019-20");
        myTrackables = myVuforia.vuforiaInit(skystoneTrackables);

        /**
         * Instantiate the drive class
         */
        DriveMecanum drive = new DriveMecanum(robot, opMode, myVuforia, myTrackables);

        /**
         * Set the initial servo positions
         */
        /**
        robot.servoRightGrab.setPosition(.5);
        robot.servoLeftGrab.setPosition(.5);
        robot.servoClawClose.setPosition(.5);
        robot.servoClawRotate.setPosition(.5);
         **/

        /**
         * Set the initial position for the Lift mechanism
         */

        /**
        robot.motorLinear.setPower(-0.3);
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
         **/

        /**
         *  Create the DataLogger object.
         */
        createDl();

        /**
         * Calibrate the gyro
         */

        /**
        robot.sensorGyro.calibrate();
        while (robot.sensorGyro.isCalibrating()) {
            telemetry.addData("Waiting on Gyro Calibration", "");
            telemetry.update();
        }

         **/
//        skystoneTrackables.activate();

        telemetry.addData("Status", "Vuforia Initialized");
        telemetry.addData(">", "System initialized and Ready");
        telemetry.update();

        /**
         * Start the opMode
         */
        waitForStart();

        while (opModeIsActive()) {
            switch (state) {
                case FOUNDATION:
                    /**
                     * Provide a description of what this state does
                     * Code goes here
                     */

                    drive.translate(.5,20, 1);
                    drive.motorsHalt();

//                    robot.servoRightGrab.setPosition(0.5);
//                    robot.servoLeftGrab.setPosition(0.5);

                    drive.translate(-1,180,.5);
                    drive.motorsHalt();
                    robot.motorRF.setPower(-0.5);
                    robot.motorRR.setPower(-0.5);
                    sleep(400);
                    drive.motorsHalt();

                    drive.translate(1, 0, .3);
                    drive.motorsHalt();

//                    robot.servoRightGrab.setPosition(0);
//                    robot.servoLeftGrab.setPosition(0);

                    state = State.SECOND_STATE;
                    //Exit the state
                    break;

                case VUFORIA_TEST:
                    /**
                     * This state is intended to test the Vuforia code and provide results to the
                     * Driver Station screen
                     */
                    /**
                    if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                        telemetry.addData("Visible Target", skystoneTrackables.getName());
                        targetVisible = true;

                        // getUpdatedRobotLocation() will return null if no new information is available since
                        // the last time that call was made, or if the trackable is not currently visible.
                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocation = robotLocationTransform;
                        }
                    }

                     **/

                    telemetry.addData("visible Target = ", visibleTarget);
                    telemetry.addData("X Coords = ", robotX);
                    telemetry.addData("Y Coords = ", robotY);
                    telemetry.addData("Robot Bearing = ", robotBearing);
                    telemetry.addData("Robot Roll = ", robotRoll);
                    telemetry.update();
//                        state = State.HALT;
                    break;

                case THIRD_STATE:
                    /**
                     * Provide a description of what this state does
                     * Code goes here
                     */

                    state = State.FOURTH_STATE;
                    break;

                case FOURTH_STATE:
                    /**
                     * Provide a description of what this state does
                     * Code goes here
                     */

                    state = State.FIFTH_STATE;
                    break;

                case FIFTH_STATE:
                    /**
                     * Provide a description of what this state does
                     * Code goes here
                     */

                    state = State.END_STATE;
                    break;

                case END_STATE:
                    /**
                     * Provide a description of what this state does
                     * Code goes here
                     */

                    state = State.HALT;
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
        telemetry.addData("Procedure", String.valueOf(procedure));
        telemetry.addData("button", String.valueOf(button));
        telemetry.addData("Heading", String.valueOf(heading));
        telemetry.addData("robotX", String.valueOf((int) robotX));
        telemetry.addData("robotY", String.valueOf((int) robotY));
        telemetry.addData("Target X", String.valueOf(x));
        telemetry.addData("Target Y", String.valueOf(y));
        telemetry.addData("robotBearing", String.valueOf((int) robotBearing));
        telemetry.addData("robotRoll", String.valueOf((int) robotBearing));
        telemetry.addData("Current Z Int", String.valueOf(currentZint));
        telemetry.addData("Z Correction", String.valueOf(zCorrection));
        telemetry.addData("touchSensor", String.valueOf(robot.touchLiftForward.getValue()));
        telemetry.addData("touchSensor", String.valueOf(robot.touchLiftBack.getValue()));
        telemetry.addData("touchSensor", String.valueOf(robot.touchLiftUp.getValue()));
        telemetry.addData("touchSensor", String.valueOf(robot.touchLiftDown.getValue()));
        telemetry.addData("ODS", String.valueOf(ods));
        telemetry.addData("Color Right Red", String.valueOf(colorRightRed));
        telemetry.addData("Color Right Blue", String.valueOf(colorRightBlue));
        telemetry.addData("Color Left Red", String.valueOf(colorLeftRed));
        telemetry.addData("Color Left Blue", String.valueOf(colorLeftBlue));
        telemetry.addData("Target Encoder Position", String.valueOf(myTargetPosition));
        telemetry.addData("Current Encoder Position", String.valueOf(robot.motorLF.getCurrentPosition()));
        telemetry.addData("LF", String.valueOf(LF));
        telemetry.addData("RF", String.valueOf(RF));
        telemetry.addData("LR", String.valueOf(LR));
        telemetry.addData("RR", String.valueOf(RR));
        telemetry.update();
    }


    /**
     * Catchall to get data from all sensor systems.  Updates global variables
     */
    private void getSensorData() {
//        ods = robot.ods.getLightDetected();
//        currentZint = robot.mrGyro.getIntegratedZValue();
        vuforiaTracking = myVuforia.getLocation(myTrackables);
        robotX = vuforiaTracking.get(0);
        robotY = vuforiaTracking.get(1);
        robotBearing = vuforiaTracking.get(2);
        robotRoll = vuforiaTracking.get(3);
        visibleTarget = vuforiaTracking.get(4);
    }

    /**
     * Setup the dataLogger
     * The dataLogger takes a set of fields defined here and sets up the file on the Android device
     * to save them to.  We then log data later throughout the class.
     */
    private void createDl() {

        Dl = new DataLogger("AutoMecanumSimpleTest" + runtime.time());
        Dl.addField("runTime ");
        Dl.addField("Alliance ");
        Dl.addField("State ");
        Dl.addField("Procedure ");
        Dl.addField("courseCorrect ");
        Dl.addField("heading ");
        Dl.addField("robotX ");
        Dl.addField("robotY ");
        Dl.addField("robotRoll ");
        Dl.addField("X ");
        Dl.addField("Y ");
        Dl.addField("robotBearing ");
        Dl.addField("visibleTarge ");
        Dl.addField("initZ ");
        Dl.addField("currentZ ");
        Dl.addField("zCorrection ");
        Dl.addField("touchSensor ");
        Dl.addField("ODS ");
        Dl.addField("colorRightRed ");
        Dl.addField("colorRightBlue ");
        Dl.addField("colorLeftRed ");
        Dl.addField("colorLeftBlue ");
        Dl.addField("LFTargetPos ");
        Dl.addField("LFMotorPos ");
        Dl.addField("motorLF ");
        Dl.addField("motorRF ");
        Dl.addField("mtoroLR ");
        Dl.addField("motorRR ");
        Dl.newLine();
    }

    /**
     * Log data to the file on the phone.
     */
    private void logData() {

        Dl.addField(String.valueOf(runtime.time()));
        Dl.addField(String.valueOf(alliance));
        Dl.addField(String.valueOf(state));
        Dl.addField(String.valueOf(procedure));
        Dl.addField(String.valueOf(courseCorrect));
        Dl.addField(String.valueOf(heading));
        Dl.addField(String.valueOf((int) robotX));
        Dl.addField(String.valueOf((int) robotY));
        Dl.addField(String.valueOf((int) robotRoll));
        Dl.addField(String.valueOf(x));
        Dl.addField(String.valueOf(y));
        Dl.addField(String.valueOf((int) robotBearing));
        Dl.addField(String.valueOf((int) visibleTarget));
        Dl.addField(String.valueOf(initZ));
        Dl.addField(String.valueOf(currentZint));
        Dl.addField(String.valueOf(zCorrection));
        Dl.addField(String.valueOf(robot.rangeSensorFront));
        Dl.addField(String.valueOf(robot.touchLiftDown.getValue()));
        Dl.addField(String.valueOf(robot.touchLiftUp.getValue()));
        Dl.addField(String.valueOf(robot.touchLiftForward.getValue()));
        Dl.addField(String.valueOf(robot.touchLiftBack.getValue()));
        Dl.addField(String.valueOf(ods));
        Dl.addField(String.valueOf(myTargetPosition));
        Dl.addField(String.valueOf(robot.motorLF.getCurrentPosition()));
        Dl.addField(String.valueOf(LF));
        Dl.addField(String.valueOf(RF));
        Dl.addField(String.valueOf(LR));
        Dl.addField(String.valueOf(RR));
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
        FOUNDATION, SECOND_STATE, THIRD_STATE, FOURTH_STATE,
        FIFTH_STATE, HALT, END_STATE, VUFORIA_TEST
    }

}
