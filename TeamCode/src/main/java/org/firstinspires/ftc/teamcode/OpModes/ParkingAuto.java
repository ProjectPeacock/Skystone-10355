/*
    Team:       10355 - Project Peacock
    Autonomous Program - Parking Only Mode
    Alliance Color: Either
    Robot Starting Position: Wherever needed.
    Strategy Description:
        - Point robot towards parking zone.  Needs to have 1 tile of space between start and stop.

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

/*
 * Import the classes we need to have local access to.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;
import org.firstinspires.ftc.teamcode.Libs.skystoneVuforia;

import java.util.List;

/*
 * Name the opMode and put it in the appropriate group
 */
@Autonomous(name = "Parking Auto", group = "COMP")

public class ParkingAuto extends LinearOpMode {

    /*
     * Instantiate all objects needed in this class
     */

    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;                     //Opmode
    private skystoneVuforia myVuforia = new skystoneVuforia();
    private List<Double> vuforiaTracking;   //List of Vuforia coordinates
    private List<VuforiaTrackable> myTrackables;    //List of Vuforia trackable objects
    private State state = State.SECOND_STATE;    //Machine State


    public void runOpMode() {
        /*
         * Setup the init state of the robot.  This configures all the hardware that is defined in
         * the HardwareTestPlatform class.
         */
        robot.init(hardwareMap);

        /*
         * Set the initial servo positions
         */
        robot.servo4Bar1.setPower(0.8);       // lower the arm to grab the stone
        robot.servo4Bar2.setPower(-0.8);
        robot.servoFoundation1.setPower(0.6);
        robot.servoFoundation2.setPower(1);
        robot.servoStone.setPower(1);
        sleep(1000);
        robot.servoSwivel.setPower(-0.4);    // Rotate the stone into position to place
        robot.servoGrab.setPower(0.3);      // be sure the stone grabber is open
        sleep(1000);
        robot.servo4Bar1.setPower(0.2);       // lower the arm to grab the stone
        robot.servo4Bar2.setPower(-0.2);
        sleep(1000);

        /*
         * Instantiate the drive class
         */
        DriveMecanum drive = new DriveMecanum(robot, opMode, myVuforia, myTrackables);

        telemetry.addData(">", "System initialized and Ready");
        telemetry.update();

        /*
         * Start the opMode
         */
        waitForStart();

        while (opModeIsActive()) {
            switch (state) {
                case FIRST_STATE:
                    state = State.SECOND_STATE;
                    //Exit the state
                    break;

                case SECOND_STATE:
                    /*
                     * This state is for testing the MR Gyro Sensor
                     * Leave the red bar facing the line
                     */
                    sleep(20000);

                    drive.translateTime(0.3, 0, 1.3);

                    state = State.HALT;
                    break;

                case HALT:
//                    drive.motorsHalt();               //Stop the motors

                    //Exit the OpMode
                    requestOpModeStop();
                    break;
            }
        }
    }

    /**
     * Enumerate the States of the machine.
     */
    enum State {
        FIRST_STATE, SECOND_STATE, HALT
    }

}
