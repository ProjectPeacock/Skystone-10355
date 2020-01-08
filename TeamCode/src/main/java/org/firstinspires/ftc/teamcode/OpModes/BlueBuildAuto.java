/*
    Team:       10355 - Project Peacock
    Autonomous Program - Blue Strategy #1 - Place Foundation and park
    Alliance Color: Blue
    Robot Starting Position: Blue build zone, wall next to build site
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

/**
 * Name the opMode and put it in the appropriate group
 */
@Autonomous(name = "Blue-Foundation, Park", group = "COMP")

public class BlueBuildAuto extends LinearOpMode {

    /**
     * Instantiate all objects needed in this class
     */

    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;                     //Opmode
    private skystoneVuforia myVuforia = new skystoneVuforia();
    private ElapsedTime runtime = new ElapsedTime();
    private double heading = 90;        //Heading for all methods
    private double y = -200;            //Vuforia y stop coordinate
    private double x = -200;            //Vuforia x stop coordinate
    private List<Double> vuforiaTracking;   //List of Vuforia coordinates
    private List<VuforiaTrackable> myTrackables;    //List of Vuforia trackable objects
    private DataLogger Dl;                          //Datalogger object
    private String alliance = "blue";                //Your current alliance
    private String courseCorrect = "";
    private State state = State.PLACE_FOUNDATION;    //Machine State


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
        telemetry.update();

        /**
         * Start the opMode
         */
        waitForStart();

        while (opModeIsActive()) {
            switch (state) {

                case PLACE_FOUNDATION:
                    /**
                     * Strafe to the foundation and move it into position
                     */

                    /**
                     * strafe diagonally to the foundation
                     */
                    drive.translateTime(.3, 165, 2.0);
                    drive.translateTime(.1, 180, 0.25);

                    /**
                     * Grab the foundation
                     */
                    robot.servoFoundation1.setPower(1);
                    robot.servoFoundation2.setPower(.6);
                    sleep(500);

                    /**
                     * drive towards the wall
                     */
                    drive.translateTime(.3,0,2);

                    /**
                     * rotate the foundation towards the wall
                     */
                    robot.motorLF.setPower(-.3);
                    robot.motorLR.setPower(-.3);
                    robot.motorRF.setPower(.3);
                    robot.motorRR.setPower(0.3);
                    sleep (1400);

                    /**
                     * drive the robot into the wall
                     */
                    drive.translateTime(0.3,180,1);

                    /**
                     * Let go of the Foundation and the stone
                     */
                    robot.servoFoundation1.setPower(0.6);
                    robot.servoFoundation2.setPower(1);
                    sleep(500);

                    /**
                     * strafe to parking position
                     */
                    drive.translateTime(.3, 25, 2.2);

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
        PLACE_FOUNDATION, LOCATE_SKYSTONE, PLACE_FIRST_STONE, GRAB_2ND_STONE,
        FIFTH_STATE, HALT, END_STATE
    }

}
