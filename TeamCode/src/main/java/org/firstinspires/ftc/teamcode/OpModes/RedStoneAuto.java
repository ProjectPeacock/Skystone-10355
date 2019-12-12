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
@Autonomous(name = "Red-Stone, Foundation, park", group = "COMP")

public class RedStoneAuto extends LinearOpMode {

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
        robot.servoGrab.setPower(-1);
        robot.servoFoundation1.setPower(0.6);
        robot.servoFoundation2.setPower(1);
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

                    /**
                     * drive towards the stone
                     * Note: The robot needs to be positioned on the second tile from the corner.
                     * The 2nd stone from the end should be centered with the grabber.
                     */

                    robot.motorRR.setPower(-0.25);
                    robot.motorRF.setPower(-0.25);
                    robot.motorLR.setPower(-0.25);
                    robot.motorLF.setPower(-0.25);
                    sleep(60);

                    /**
                     * Raise the lifting mechanism to position the grabber to grab the stone.
                     * This occurs while the robot is still moving forward.
                     */

                    robot.motorLinear.setPower(0.3);
                    sleep(800);
                    robot.motorLinear.setPower(0);

                    /**
                     * Stop all motors.  This should place the robot right next to the stone we
                     * want to grab.
                     * Pause to allow the robot to stop moving and the stone to settle.
                     */
                    drive.motorsHalt();
                    sleep(500);

                    /**
                     * Grab the block with the grabber.
                     */
                    robot.servoGrab.setPower(0);
                    sleep(1000);

                    /**
                     * Lay the lifting mechanism down so that we can pass under the bar.
                     */
                    robot.motorLinear.setPower(-0.3);
                    sleep(300);

                    /**
                     * Back the robot up so that we can avoid the skybridge.
                     */
                    robot.motorRF.setPower(0.2);
                    robot.motorRR.setPower(0.2);
                    robot.motorLR.setPower(0.2);
                    robot.motorLF.setPower(0.2);
                    sleep(600);
                    drive.motorsHalt();

                    /**
                     * Strafe to the foundation.
                     */
                    robot.motorRF.setPower(0.3);
                    robot.motorRR.setPower(-0.3);
                    robot.motorLR.setPower(0.3);
                    robot.motorLF.setPower(-0.3);
                    sleep(2900);
                    drive.motorsHalt();

                    /**
                     * Adjust the robot so that the left side is more parallel with the foundation.
                     * This step is not needed if the gyro sensor is in place as the robot should
                     * strafe without sway or yaw.  This is a temporary fix.
                     */
                    robot.motorLF.setPower(-0.2);
                    robot.motorLR.setPower(-0.2);
                    sleep(300);
                    drive.motorsHalt();

                    /**
                     * Drive the robot forward into the foundation so that we can grab it.
                     */
                    robot.motorRF.setPower(-0.2);
                    robot.motorRR.setPower(-0.2);
                    robot.motorLR.setPower(-0.2);
                    robot.motorLF.setPower(-0.2);
                    sleep(500);
                    drive.motorsHalt();

                    /**
                     * Engage the tractor beam! (i.e. grab the foundation)
                     */
                    robot.servoFoundation1.setPower(1);
                    robot.servoFoundation2.setPower(.6);
                    sleep(500);

                    /**
                     * Pull the foundation towards the build site, arcing so that the long side of
                     * the foundation will be parallel to the back wall.
                     */
                    robot.motorLR.setPower(0.3);
                    robot.motorLF.setPower(0.3);
                    robot.motorRR.setPower(0.15);
                    robot.motorRF.setPower(0.15);
                    sleep (2100);
                    drive.motorsHalt();

                    /**
                     * Push the foundation into the wall.
                     */
                    robot.motorLR.setPower(-0.2);
                    robot.motorLF.setPower(-0.2);
                    robot.motorRR.setPower(-0.2);
                    robot.motorRF.setPower(-0.2);
                    sleep (1900);
                    drive.motorsHalt();

                    /**
                     * Deposit the payload onto the foundation.
                     */
                    robot.servoGrab.setPower(-0.5);
                    sleep(500);

                    /**
                     * return the grabber to original position for teleop mode.
                     */
                    robot.servoGrab.setPower(0);
                    sleep(500);

                    /**
                     * Release the tractor beam. (i.e.let go of the Foundation.)
                     */
                    robot.servoFoundation1.setPower(0.6);
                    robot.servoFoundation2.setPower(1);
                    sleep(500);

                    /**
                     * Move to the parking position
                     */
                    robot.motorLR.setPower(0.3);
                    robot.motorLF.setPower(0.0);
                    robot.motorRR.setPower(0.0);
                    robot.motorRF.setPower(0.3);
                    sleep (1900);
                    drive.motorsHalt();

                    /**
                     * Strafe towards the wall to get out of the way of the partner robot.
                     */
                    robot.motorLR.setPower(0.1);
                    robot.motorLF.setPower(-0.1);
                    robot.motorRR.setPower(-0.2);
                    robot.motorRF.setPower(0.2);
                    sleep (1500);
                    drive.motorsHalt();


                    /**
                     * Close out the program.
                     */
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