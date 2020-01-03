/*
    Team:       10355 - Project Peacock
    Presentation Program - Will only be used to demonstrate capabilities of the robot during
          presentations.

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
 **/

package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;

/**
 *
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Presentation Teleop", group = "Comp")
@Disabled

public class PTeleOp extends LinearOpMode {
    /**
     * Instantiate all objects needed in this class
     */
    private final static HardwareProfile robot = new HardwareProfile();

    @Override
    public void runOpMode() {

        begin();                    // initialize the robot's hardware

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Project Peacock");    //
        telemetry.update();

        // Wait for the user to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("COMP OpModeMecanum Active", "");    //
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /**
             *
             * Driving algorithm
             * All driving algorithm items have been removed from this program.
             *
             **/

            /**
             * Code for controlling the foundation grabber
             **/

            if (gamepad2.dpad_down){
                robot.servoFoundation1.setPower(1);
                robot.servoFoundation2.setPower(.6);
            } if (gamepad2.dpad_up){
                robot.servoFoundation1.setPower(0.6);
                robot.servoFoundation2.setPower(1);
            }

            /**
             * Code to manually control linear leaning mechanism
             **/

            if (gamepad2.right_stick_y < -0.3 && robot.touchLiftForward.isPressed()== false){ // Analog stick pointing up for going up (Mechanism Control)
                robot.motorLinear.setPower(-1 * gamepad2.right_stick_y* 0.125);
            }
            else if  (gamepad2.right_stick_y > 0.3 && robot.touchLiftBack.isPressed()== false){ // analog stick down for going down (Mechanism Control)
                robot.motorLinear.setPower(-1 *gamepad2.right_stick_y* 0.125);
            }
            else robot.motorLinear.setPower(0);

            /**
             *
             * Code to manually control lift mechanism lifting
             *
             **/
            if (gamepad2.right_trigger > 0){ // Change To Right-Trigger (Mechanism Control)
                robot.motorLift.setPower(0.5);
            }
            else if (gamepad2.left_trigger > 0){ // Change to Left_Trigger (mechanism Control)
                robot.motorLift.setPower(-0.5);
            }
            else {
                robot.motorLift.setPower(0);
            }

            /**
             *  Code to control the 4-bar mechanism
             */

            if (gamepad2.left_stick_y < -0.2){
                robot.motor4Bar.setPower(-gamepad2.left_stick_y);
            }
            else if(gamepad2.left_stick_y > 0.2){
                robot.motor4Bar.setPower(-gamepad2.left_stick_y);
            }
            else{
                robot.motor4Bar.setPower(0);
            }

            /**
             * Code to control grab mechanism
             **/
            if (gamepad2.b == true){
                robot.servoGrab.setPower(-0.5);
            }
            else {
                robot.servoGrab.setPower(0);
            }

            idle();

        }

    }

    private void begin() {

        /**
         * Initialize the robot's hardware.  The hardware configuration can be found in the
         * Hardware/HardwareProfile.java class.
         */
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

}