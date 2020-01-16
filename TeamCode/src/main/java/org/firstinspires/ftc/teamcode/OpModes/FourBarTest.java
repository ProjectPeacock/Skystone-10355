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
 **/

package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Libs.PIDController;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "4-Bar Test", group = "Test")

public class FourBarTest extends LinearOpMode {
    /*
     * Instantiate all objects needed in this class
     */
    private final static HardwareProfile robot = new HardwareProfile();
    // our DC motor.
//    DcMotor                 leftMotor, rightMotor;
    double                  globalAngle, power = .30, correction, rotation;
    boolean                 aButton, bButton, touched;
    PIDController           pidRotate;
    public static final double NEW_P = 2.5;
    public static final double NEW_I = 0.1;
    public static final double NEW_D = 0.2;

    @Override
    public void runOpMode() {
        double v1;
        double v2;
        double v3;
        double v4;
        double r;
        double robotAngle;
        double rightX;
        double rightY;
        double powerLevel = 0.5;

        double speedFactor = 1;

        begin();                    // initialize the robot's hardware

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Project Peacock");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDController(.003, .00003, 0);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        /*
         *
         *
         *
         *
         */
        telemetry.addData("COMP OpModeMecanum Active", "");    //
        telemetry.update();
        // run until the end of the match (driver presses STOP)


        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDCoefficients pidOrig = robot.motor4Bar.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // change coefficients using methods included with DcMotorEx class.
        PIDCoefficients pidNew = new PIDCoefficients(NEW_P, NEW_I, NEW_D);
        robot.motor4Bar.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        // re-read coefficients and verify change.
        PIDCoefficients pidModified = robot.motor4Bar.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // display info to user.

        while (opModeIsActive()) {



//            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.addData("4 turn rotation", rotation);
            telemetry.update();

            // set power levels.
            robot.motor4Bar.setPower(power - correction);

            // We record the sensor values because we will test them in more than
            // one place with time passing between those places. See the lesson on
            // Timing Considerations to know why.

            aButton = gamepad1.a;
            bButton = gamepad1.b;

            if (touched || aButton || bButton)
            {
                // backup.
                robot.motor4Bar.setPower(-power);

                sleep(500);

                // stop.
                robot.motor4Bar.setPower(0);

                // turn 90 degrees right.
                if (touched || aButton) rotate(-90, power);

                // turn 90 degrees left.
                if (bButton) rotate(90, power);
            }
        }

        // turn the motors off.
        robot.motor4Bar.setPower(0);

        /*
         * Change to Slow Mode
         * Chassis Control
         */
        if (gamepad1.a){
            sleep(100);
            speedFactor = 0.5;      // reduce the speed
        }

        /*
         * Change to Fast Mode
         * Chassis Control
         */
        if (gamepad1.b){
            sleep(100);
            speedFactor = 1;    // set to full speed
        }

        /*
         * Driving algorithm
         * Note: this algorithm assumes that all values are zero when controls are not touched
         * Chassis Control
         */
        robotAngle = Math.atan2(gamepad1.left_stick_y, (gamepad1.left_stick_x * -1)) - Math.PI / 4;
        rightX = gamepad1.right_stick_x;
        rightY = gamepad1.right_stick_y;
        r = Math.hypot((gamepad1.left_stick_x * -1), gamepad1.left_stick_y);
        v1 = (r * Math.cos(robotAngle) + rightX + rightY) * powerLevel;
        v2 = (r * Math.sin(robotAngle) - rightX + rightY) * powerLevel;
        v3 = (r * Math.sin(robotAngle) + rightX + rightY) * powerLevel;
        v4 = (r * Math.cos(robotAngle) - rightX + rightY) * powerLevel;

        robot.motorLF.setPower(v1 * speedFactor);
        robot.motorRF.setPower(v2 * speedFactor);
        robot.motorLR.setPower(v3 * speedFactor);
        robot.motorRR.setPower(v4 * speedFactor);

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
            robot.motorLinear.setPower(-1 * gamepad2.right_stick_y* 0.500);
        }
        else if  (gamepad2.right_stick_y > 0.3 && !robot.touchLiftBack.isPressed()){
            robot.motorLinear.setPower(-1 *gamepad2.right_stick_y* 0.300);
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
            robot.servoGrab.setPower(-0.5); // opens the grabber only when required
        } else {
            robot.servoGrab.setPower(0.2);  // automatically closes grabber
        }

        idle();
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
    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.


//        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        double deltaAngle = 0;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

//        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        // restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0)
            {
                robot.motor4Bar.setPower(-power);
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                robot.motor4Bar.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                robot.motor4Bar.setPower(-power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        robot.motor4Bar.setPower(0);

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }

}