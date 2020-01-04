/*
    Team:       10355 - Project Peacock
    Autonomous Program - Red Strategy #1
    Actions: Place Foundation and Park
    Alliance Color: Red
    Robot Starting Position: Build zone, next to build site
    Strategy Description:
        - Grab foundation and place it in the build site
        - under the skybridge
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

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;
import org.firstinspires.ftc.teamcode.Libs.DataLogger;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;
import org.firstinspires.ftc.teamcode.Libs.skystoneVuforia;

import java.util.List;

/**
 * Name the opMode and put it in the appropriate group
 */
@Autonomous(name = "Red-Foundation, Park", group = "COMP")

public class RedBuildAuto extends LinearOpMode {

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
    private State state = State.FIRST_STATE;    //Machine State
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
        robot.servoFoundation1.setPower(0.6);
        robot.servoFoundation2.setPower(1);
        robot.servoGrab.setPower(-1);
        sleep(1000);


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
                    robot.motorLR.setPower(0.0);
                    robot.motorLF.setPower(-0.4);
                    robot.motorRR.setPower(-0.4);
                    robot.motorRF.setPower(0.0);
                    sleep (1200);
                    drive.motorsHalt();

                    /**
                     * Grab the foundation
                     */
                    robot.servoFoundation1.setPower(1);
                    robot.servoFoundation2.setPower(.6);
                    sleep(500);

                    robot.motorLR.setPower(0.3);
                    robot.motorLF.setPower(0.3);
                    robot.motorRR.setPower(0.1);
                    robot.motorRF.setPower(0.1);
                    sleep (2500);
                    drive.motorsHalt();

                    robot.motorLR.setPower(-0.2);
                    robot.motorLF.setPower(-0.2);
                    robot.motorRR.setPower(-0.2);
                    robot.motorRF.setPower(-0.2);
                    sleep (1500);
                    drive.motorsHalt();



                    /**
                     * Let go of the Foundation
                     */
                    robot.servoFoundation1.setPower(0.6);
                    robot.servoFoundation2.setPower(1);
                    sleep(500);

                    robot.motorLR.setPower(0.3);
                    robot.motorLF.setPower(0.0);
                    robot.motorRR.setPower(0.0);
                    robot.motorRF.setPower(0.3);
                    sleep (1700);
                    drive.motorsHalt();

                    robot.motorLR.setPower(0.1);
                    robot.motorLF.setPower(-0.1);
                    robot.motorRR.setPower(-0.2);
                    robot.motorRF.setPower(0.2);
                    sleep (1500);
                    drive.motorsHalt();
//                    drive.translate(0.5,90,0.5);
//                    drive.translate(-1,90,1);

                    /**
                     *Drive forwards, lower servo,
                     * drive back, to the left,
                     * push base into the corner, and park
                     */

//                    drive.translate(4,90, 2);

                   /** robot.motorLR.setPower(0.0);
                    robot.motorLF.setPower(0.3);
                    robot.motorRR.setPower(0.3);
                    robot.motorRF.setPower(0.0);
                    sleep (1500);
                    drive.motorsHalt();

                    /**
                     * Grab the foundation
                     */
                   /** robot.servoFoundation1.setPower(1);
                    robot.servoFoundation2.setPower(.6);
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

                    /**
                     * Let go of the foundation
                     */
                  /**  robot.servoFoundation1.setPower(0.6);
                    robot.servoFoundation2.setPower(1);
                    sleep(500);**/

                  /**  robot.motorLR.setPower(-0.0);
                    robot.motorLF.setPower(-0.3);
                    robot.motorRR.setPower(-0.3);
                    robot.motorRF.setPower(-0.0);
                    sleep (1000);
                    drive.motorsHalt();

//                    drive.translate(0.5,90,0.5);
//                    drive.translate(-1,90,1);**/

                    state = State.HALT;
                    //Exit the state
                    break;

                case SECOND_STATE:
                    /**
                     * This state is for testing the MR Gyro Sensor
                     */

                    drive.translateTime(0.3, 30, 2);
                    currentZint = robot.mrGyro.getIntegratedZValue();
                    telemetry.addData("Heading = ", currentZint);
                    telemetry.update();

                    robot.servoRightGrab.setPosition(0.4);
                    sleep(500);

                    robot.motorLR.setPower(-0.1);
                    robot.motorLF.setPower(-0.1);
                    robot.motorRR.setPower(-0.3);
                    robot.motorRF.setPower(-0.3);
                    sleep (1500);
                    drive.motorsHalt();

                    drive.translateTime(0.2, 0, 1);

                    robot.servoRightGrab.setPosition(.9);
                    sleep(500);

                    drive.translateTime(0.3, 210, 1);
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
     * Enumerate the States of the machine.
     */
    enum State {
        FIRST_STATE, SECOND_STATE, THIRD_STATE, FOURTH_STATE,
        FIFTH_STATE, HALT, END_STATE
    }

}