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
        - Grabbing mechanism - controlled by two continuous rotation servos (1 for grabbing, 1 for swiveling)
        - Foundation Grabbing mechanism - controlled by two continuous rotation servos (one for
             each side of the robot)
        - Virtual 4-bar mechanism - Controlled by two servos. Allows for extension and
             positioning of the stone.
        - Lifting mechanism - Controlled by 1 motor with encoder. Lifts the placement system.
        - Delivery mechanism - Controlled 1 continuous rotation servo. Moves stones from intake
             mechanism to the placement mechanism.
        - Intake mechanism - Controlled by 2 goBuilda 312 RPM motors.
        - 2 x Touch sensors - limits lift mechanism when leaning forward and backward.
 */

package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;

import java.util.List;
import java.util.Locale;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Stack Bot", group = "Comp")

public class grabBot extends LinearOpMode {
    /*
     * Instantiate all objects needed in this class
     */
    private final static HardwareProfile robot = new HardwareProfile();
    private double v1 = 0;
    private double v2 = 0;
    private double v3 = 0;
    private double v4 = 0;
    private double v1s = 0;
    private double v2s = 0;
    private double v3s = 0;
    private double v4s = 0;
    private double r = 0;
    private double robotAngle =0;
    private double rightX = 0;
    private double rightY = 0;
    private double powerLevel = 0.5;
    private double modePower = 0.5;
    private String dpadDownState = "play";
    private String dpadUpState = "play";
    private double initTime;
    private double elapsedTime;

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
                modePower = 0.5;
            }

            /*
             * Change to Fast Mode
             * Chassis Control
             */
            if (gamepad1.b){
                sleep(100);
                modePower = 1;
            }

            /*
             *
             * Driving algorithm
             * Note: this algorithm assumes that all values are zero when controls are not touched
             * Chassis Control
             */

            //Calculate the power needed for each motor
            //            fwdControl = -1 * gamepad1.left_stick_y;
            robotAngle = Math.atan2(gamepad1.left_stick_y, (-gamepad1.left_stick_x)) - Math.PI / 4;
            rightX = gamepad1.right_stick_x;
            rightY = -gamepad1.right_stick_y;
            r = -Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            v1 = (r * Math.cos(robotAngle) + rightX + rightY) * powerLevel;
            v2 = (r * Math.sin(robotAngle) - rightX + rightY) * powerLevel;
            v3 = (r * Math.sin(robotAngle) + rightX + rightY) * powerLevel;
            v4 = (r * Math.cos(robotAngle) - rightX + rightY) * powerLevel;

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
            if (gamepad1.dpad_down){
                robot.servoFoundation1.setPower(0.9);
                robot.servoFoundation2.setPower(0.5);
            }
            if (gamepad1.dpad_up){
                robot.servoFoundation1.setPower(0.5);
                robot.servoFoundation2.setPower(0.9);
            }

            /*
             * Code for controlling the intake system
             */
            if (gamepad1.right_trigger != 0){
                robot.motorIntake1.setPower(-gamepad1.right_trigger);
                robot.motorIntake2.setPower(gamepad1.right_trigger);
                robot.servoDelivery.setPower(-1);
            } else if (gamepad1.left_trigger != 0){
                robot.motorIntake1.setPower(gamepad1.left_trigger);
                robot.motorIntake2.setPower(-gamepad1.left_trigger);
                robot.servoDelivery.setPower(1);
            } else {
                robot.motorIntake1.setPower(0);
                robot.motorIntake2.setPower(0);
                robot.servoDelivery.setPower(0);
            }

            /*
             * Code for stone placement mechanism
             */
            if ((gamepad2.dpad_down || !dpadDownState.equals("play")) && dpadUpState.equals("play")){
                if (dpadDownState.equals("play")){
                    robot.servoSwivel.setPower(-0.4);
                    robot.servoGrab.setPower(0.90);
                    robot.servo4Bar1.setPower(0.8);       // raise arm to reset swivel and grabber
                    robot.servo4Bar2.setPower(-0.8);
                    initTime = getRuntime();
                    dpadDownState = "raise";
                }
                if(dpadDownState.equals("raise")){
                    if (!robot.touchLiftForward.isPressed()){
                        robot.motorLinear.setPower(0.4);
                    } else {
                        robot.motorLinear.setPower(0);
                        elapsedTime = getRuntime() - initTime; // allow time for the to swivel
                        if (elapsedTime >= 0.500) {
                            dpadDownState = "lower";
                            robot.servoGrab.setPower(0.25);
                            initTime = getRuntime();
                        }
                    }
                }
                if (dpadDownState.equals("lower")) {
                    robot.servo4Bar1.setPower(-0.1);       // lower the arm to grab the stone
                    robot.servo4Bar2.setPower(0.1);
                    elapsedTime = getRuntime() - initTime; // allow time for the arm to lower before grabbing the stone
                    if (elapsedTime >= 0.400){
                        dpadDownState = "grab";
                    }
                }
                if (dpadDownState.equals("grab")) {
                    robot.servoGrab.setPower(0.90);    // move to position to grab the stone
                    dpadDownState = "play";         // let the system know that the stone is ready to be placed
                }
            }

            if ((gamepad2.dpad_up || !dpadUpState.equals("play")) && dpadDownState.equals("play")){
                if (dpadUpState.equals("play")) {
                    robot.motorLift.setPower(-1);
//                    robot.servoSwivel.setPower(-0.4);    // Rotate the stone into position to place
                    robot.servoGrab.setPower(0.90);      // be sure the stone grabber is open
                    dpadUpState = "raise";
                    initTime = getRuntime();
                }
                if (dpadUpState.equals("raise")) {
                    elapsedTime = getRuntime() - initTime; // allow time for the arm to lower before grabbing the stone
                    robot.servo4Bar1.setPower(0.8);       // lower the arm to grab the stone
                    robot.servo4Bar2.setPower(-0.8);
                    if (elapsedTime >= 0.4){
                        robot.motorLift.setPower(0);
                        dpadUpState = "play";
                        robot.servoSwivel.setPower(1);    // Rotate the stone into position to place
                    }
                }
            }

            /*
             * Code to manually control linear leaning mechanism
             * Mechanism Control
             */
            if (gamepad2.right_stick_y < -0.1 && !robot.touchLiftForward.isPressed()){
                robot.motorLinear.setPower(-1 * gamepad2.right_stick_y);
                robot.servo4Bar1.setPower(0.2);       // lower the arm to grab the stone
                robot.servo4Bar2.setPower(-0.2);
            }
            else if  (gamepad2.right_stick_y > 0.1 && !robot.touchLiftBack.isPressed()){
                robot.servoSwivel.setPower(-0.4);    // Rotate the stone into position to place
                robot.servoGrab.setPower(0.25);      // be sure the stone grabber is open
                robot.motorLinear.setPower(-0.3 *gamepad2.right_stick_y);
                robot.servo4Bar1.setPower(0.2);       // lower the arm to grab the stone
                robot.servo4Bar2.setPower(-0.2);
            } else if (dpadDownState != "play"){
                //do nothing so that the dpad button can control the operation of the linear motor
            } else robot.motorLinear.setPower(0);

            /*
             * Code to manually control lift mechanism lifting
             * Mechanism Control
             */
            if (gamepad2.right_trigger > 0){
                robot.motorLift.setPower(0.5);
            }
            else if (gamepad2.left_trigger > 0){
                robot.motorLift.setPower(-0.3);
            }else if(!dpadUpState.equals("play")){

            } else {
                robot.motorLift.setPower(0);
            }

            /*
             *  Code to control the capstone release
             */
            if (gamepad2.left_bumper || gamepad1.right_bumper) {
                robot.servoCapstone.setPosition(-0.5);
            }

            if (gamepad1.x){
                robot.servoStone.setPower(-1);
            } else {
                robot.servoStone.setPower(1);
            }

            /*
             *  Code to control the 4-bar mechanism
             *  Mechanism Control
             */
            if (gamepad2.left_stick_y != 0){
                robot.servo4Bar1.setPower(-gamepad2.left_stick_y);
                robot.servo4Bar2.setPower(gamepad2.left_stick_y);
            }

            /*
             * Code to control the grab servo
             */
            if (gamepad2.a) {
                robot.servoGrab.setPower(0.90);
            }
            if(gamepad2.b) {
                robot.servoGrab.setPower(0.25);
            }

            /*
             * Rotate grabber to place the stone perpendicular to the foundation
             */
            if (gamepad2.x){
                robot.servoSwivel.setPower(1);
            }

            /*
             * Rotate grabber to place the stone parallel to the foundation
             */
            if (gamepad2.y){
                robot.servoSwivel.setPower(0.3);
            }

            /*
             * Capstone release
             */

            if (gamepad2.left_bumper){
                robot.servoCapstone.setPosition(0.5);
            }

            idle();
            telemetry.addData("dpadUpState = ", dpadUpState);
            telemetry.addData("dpadDownState = ", dpadDownState);
            telemetry.addData("initialization Time = ", initTime);
            telemetry.addData("Currnet Runtime", getRuntime());
            telemetry.addData("Elapsed Time = ", elapsedTime);
            telemetry.update();
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