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

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;
import org.firstinspires.ftc.teamcode.Libs.DataLogger;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;
import org.firstinspires.ftc.teamcode.Libs.skystoneVuforia;

import java.util.List;

/**
 * Name the opMode and put it in the appropriate group
 */
@Autonomous(name = "Blue Build Zone", group = "COMP")

public class BlueBuildAuto1 extends LinearOpMode {

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
    private State state = State.SECOND_STATE;    //Machine State
    private boolean initialize = false;


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
        robot.servoRightGrab.setPosition(0.9);
        //robot.servoLeftGrab.setPosition(.5);
        robot.servoClawClose.setPosition(.5);
        //robot.servoClawRotate.setPosition(.5);


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
        telemetry.update();

        /**
         * Start the opMode
         */
        waitForStart();

        while (opModeIsActive()) {
            switch (state) {
                case FIRST_STATE:
                    /**
                     *Drive forwards, lower servo,
                     * drive back, to the left,
                     * push base into the corner, and park
                     */

//                    drive.translate(4,90, 2);

                    robot.motorLR.setPower(0.0);
                    robot.motorLF.setPower(0.3);
                    robot.motorRR.setPower(0.3);
                    robot.motorRF.setPower(0.0);
                    sleep (1500);
                    drive.motorsHalt();

                    robot.servoRightGrab.setPosition(0.4);
                    sleep(500);

                    robot.motorLR.setPower(-0.1);
                    robot.motorLF.setPower(-0.1);
                    robot.motorRR.setPower(-0.3);
                    robot.motorRF.setPower(-0.3);
                    sleep (1000);
                    drive.motorsHalt();

                    robot.motorLR.setPower(0.2);
                    robot.motorLF.setPower(0.2);
                    robot.motorRR.setPower(0.2);
                    robot.motorRF.setPower(0.2);
                    sleep (1000);
                    drive.motorsHalt();

//                    drive.translate(-1,20,2); //if 20 heading doesn't work try 110

                    robot.servoRightGrab.setPosition(.9);
                    sleep(500);

                    robot.motorLR.setPower(-0.0);
                    robot.motorLF.setPower(-0.3);
                    robot.motorRR.setPower(-0.3);
                    robot.motorRF.setPower(-0.0);
                    sleep (1000);
                    drive.motorsHalt();

//                    drive.translate(0.5,90,0.5);
//                    drive.translate(-1,90,1);

                    state = State.HALT;
                    //Exit the state
                    break;

                case SECOND_STATE:
                    /**
                     * This state is for testing the MR Gyro Sensor
                     */

                    drive.translateTime(0.3, 330, 1.8);
                    currentZint = robot.mrGyro.getIntegratedZValue();
                    telemetry.addData("Heading = ", currentZint);
                    telemetry.update();

                    robot.servoRightGrab.setPosition(0.4);
                    sleep(500);


                    robot.motorLR.setPower(-0.2);
                    robot.motorLF.setPower(-0.2);
                    robot.motorRR.setPower(-0.4);
                    robot.motorRF.setPower(-0.4);
                    sleep (1400);
                    drive.motorsHalt();

                    robot.motorLR.setPower(-0.4);
                    robot.motorLF.setPower(-0.4);
                    robot.motorRR.setPower(0.4);
                    robot.motorRF.setPower(0.4);
                    sleep (1500);
                    drive.motorsHalt();

                    drive.translateTime(0.3, 0, 2);
                    robot.servoRightGrab.setPosition(.9);
                    sleep(500);

                    robot.motorLR.setPower(-0.2);
                    robot.motorLF.setPower(-0.2);
                    robot.motorRR.setPower(-0.2);
                    robot.motorRF.setPower(-0.2);
                    sleep(1750);


//                    drive.translateTime(0.3, 210, 2);
                    state = State.HALT;
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
//                    drive.motorsHalt();               //Stop the motors

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
        Dl.addField(String.valueOf(procedure));
        Dl.addField(String.valueOf(courseCorrect));
        Dl.addField(String.valueOf(heading));
        Dl.addField(String.valueOf((int) robotX));
        Dl.addField(String.valueOf((int) robotY));
        Dl.addField(String.valueOf(x));
        Dl.addField(String.valueOf(y));
        Dl.addField(String.valueOf((int) robotBearing));
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
        FIRST_STATE, SECOND_STATE, THIRD_STATE, FOURTH_STATE,
        FIFTH_STATE, HALT, END_STATE
    }

}
