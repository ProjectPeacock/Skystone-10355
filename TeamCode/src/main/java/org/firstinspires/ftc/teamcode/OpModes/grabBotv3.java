/*
    Team:       10355 - Project Peacock
    grabBot Program
    Alliance Color: Either
    Robot Starting Position: Should be in the parked zone.
    Strategy Description:
        - Can operated as a miner
        - Can operate as a builder

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
 */

package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "grabBot3", group = "Comp")

public class grabBotv3 extends LinearOpMode {
    /*
     * Instantiate all objects needed in this class
     */
    private final static HardwareProfile robot = new HardwareProfile();
    private double v1 = 0;
    private double v2 = 0;
    private double v3 = 0;
    private double v4 = 0;
    private double r = 0;
    private double robotAngle =0;
    private double rightX = 0;
    private double rightY = 0;
    private double powerLevel = 0.5;
    private double modePower = 1.0;
    private double max;
    private boolean fastMode = true;

    @Override
    public void runOpMode() {

        begin();                    // initialize the robot's hardware

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);


        robot.servoStone.setPower(-1);
        sleep(300);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Project Peacock");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("COMP OpModeMecanum Active", "");    //
        telemetry.update();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            /*
             * Change to Slow Mode
             * Chassis Control
             */
            if (gamepad1.a){
                sleep(100);
                fastMode = false;
                modePower = 0.5;
            }

            /*
             * Change to Fast Mode
             * Chassis Control
             */
            if (gamepad1.b){
                sleep(100);
                fastMode = true;
                modePower = 1.0;
            }

            /*
             *
             * Driving algorithm
             * Note: this algorithm assumes that all values are zero when controls are not touched
             * Chassis Control
             * Fast Mode
             */
                    //Calculate the power needed for each motor
                    //            fwdControl = -1 * gamepad1.left_stick_y;
                    robotAngle = Math.atan2(gamepad1.left_stick_y, (gamepad1.left_stick_x * -1)) - Math.PI / 4;
                    rightX = gamepad1.right_stick_x;
                    rightY = gamepad1.right_stick_y;
                    r = Math.hypot((gamepad1.left_stick_x * -1), gamepad1.left_stick_y);
                    v1 = (r * Math.cos(robotAngle) + rightX + rightY) * powerLevel;
                    v2 = (r * Math.sin(robotAngle) - rightX + rightY) * powerLevel;
                    v3 = (r * Math.sin(robotAngle) + rightX + rightY) * powerLevel;
                    v4 = (r * Math.cos(robotAngle) - rightX + rightY) * powerLevel;

                    max = Math.max(Math.max(Math.max(Math.abs(v1), Math.abs(v2)),Math.abs(v3)),Math.abs(v4));

                    if (max > 1) {
                        v1 = v1 / max;
                        v2 = v2 / max;
                        v3 = v3 / max;
                        v4 = v4 / max;
                    }

                    robot.motorLF.setPower(v1 * modePower);
                    robot.motorRF.setPower(v2 * modePower);
                    robot.motorLR.setPower(v3 * modePower);
                    robot.motorRR.setPower(v4 * modePower);

            /* Chassis control is for jameson meaning it should all be the way jameson wants it */
            /* Mechanism control is for Julian therefore controls should be to his preference */

            /*
             * Code for controlling the foundation grabber
             * Chassis Control
             */

            if (gamepad2.dpad_down || gamepad1.dpad_down){
                robot.servoFoundation1.setPower(1);
                robot.servoFoundation2.setPower(.6);
            } if (gamepad2.dpad_up || gamepad1.dpad_up){
                robot.servoFoundation1.setPower(0.6);
                robot.servoFoundation2.setPower(1);
            }

            /*
             * Code to manually control linear leaning mechanism
             * Mechanism Control
             */

            if (gamepad2.right_stick_y < -0.3 && !robot.touchLiftForward.isPressed()){
                robot.motorLinear.setPower(-1 * gamepad2.right_stick_y);
            }
            else if  (gamepad2.right_stick_y > 0.3 && !robot.touchLiftBack.isPressed()){
                robot.motorLinear.setPower(-1 *gamepad2.right_stick_y);
            }
            else robot.motorLinear.setPower(0);

            /*
             * Code to manually control lift mechanism lifting
             * Mechanism Control
             */
            if (gamepad2.right_trigger > 0){
                robot.motorLift.setPower(0.5);
            }
            else if (gamepad2.left_trigger > 0){
                robot.motorLift.setPower(-0.5);
            }
            else {
                robot.motorLift.setPower(0);
            }

            /*
             *  Code to control the captone release
             */
            if (gamepad2.left_bumper || gamepad2.right_bumper || gamepad1.left_bumper || gamepad1.right_bumper) {
                robot.servoStone.setPower(-0.5);
            }

            /*
             *  Code to control the 4-bar mechanism
             *  Mechanism Control
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

            /*
             * Code to control grab mechanism
             * Mechanism Control
             */
            if (gamepad2.b){
                robot.servoGrab.setPower(-0.5);
            }

            else {
                robot.servoGrab.setPower(0.2);
            }

            /*
             * Code to control Intake
             * Mechanism Control
             */

            idle();
         }

    }


    private void begin() {

        /*
         * Initialize the robot's hardware.  The hardware configuration can be found in the
         * HardwareTestPlatform.java class.
         */
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
}