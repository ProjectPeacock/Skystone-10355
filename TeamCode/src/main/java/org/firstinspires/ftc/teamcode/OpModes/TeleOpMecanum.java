/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.HardwareTestPlatform;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 * <p>
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Test TeleOpMecanum", group = "Test")

public class TeleOpMecanum extends LinearOpMode {
    /**
     * Instantiate all objects needed in this class
     */
    private final static HardwareTestPlatform robot = new HardwareTestPlatform();

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

            //Get the values from the gamepads
            double armPower = 0;

//            double r = Math.hypot((gamepad1.left_stick_x * -1), gamepad1.left_stick_y);
            double fwdControl = -1 * gamepad1.left_stick_y;
            double strafeControl = gamepad1.left_stick_x;
/*            double robotAngle = Math.atan2(gamepad1.left_stick_y, (gamepad1.left_stick_x * -1)) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;
*/


            idle();

            if (gamepad2.right_trigger != 0) {
                shoot();
            }


            if((gamepad2.left_trigger != 0) || (gamepad1.left_trigger!=0)){
                robot.motorFeeder.setPower(-.4);
                robot.servoInTake.setPosition(.3);
            } else {
//                robot.motorFeeder.setPower(0);
                robot.servoInTake.setPosition(1);
            }

            if(gamepad2.left_bumper == true){
                robot.servoBallBumper.setPosition(.9);
            } else {
                robot.servoBallBumper.setPosition(0.2);
            }

            if(gamepad2.b == true){         //      <= may want to remove
                // do nothing
            } else {
                // do nothing
            }

            if(gamepad2.right_bumper == true){
                robot.servoFeeder.setPosition(0.2);
            } else {
                robot.servoFeeder.setPosition(0.5);
            }

            // Disable the shooter
            if(gamepad1.a == true) {
                robot.motorShooter.setPower(1);
                while(robot.touchSensor.isPressed() == true){
                    // sit and wait
                }
                robot.motorShooter.setPower(0);
            }

            //Calculate the power needed for each motor
            if ((gamepad1.right_stick_y !=0) || (gamepad1.right_stick_x !=0)){
                robot.motorLF.setPower((-1* gamepad1.right_stick_y) + gamepad1.right_stick_x);
                robot.motorRF.setPower((-1 * gamepad1.right_stick_y) - gamepad1.right_stick_x);
                robot.motorLR.setPower((-1 * gamepad1.right_stick_y) + gamepad1.right_stick_x);
                robot.motorRR.setPower((-1 *gamepad1.right_stick_y) - gamepad1.right_stick_x);

            } else if ((gamepad1.left_stick_x !=0) || (gamepad1.left_stick_y !=0)){
                robot.motorLF.setPower(fwdControl * 0.82 + strafeControl);
                robot.motorRF.setPower(fwdControl - strafeControl * 0.82);
                robot.motorLR.setPower(fwdControl - strafeControl * 0.82);
                robot.motorRR.setPower(fwdControl * 0.82 + strafeControl);

            } else if (gamepad1.dpad_right == true) {
                robot.motorLF.setPower(1);
                robot.motorRF.setPower(-1);
                robot.motorLR.setPower(-1);
                robot.motorRR.setPower(1);

            } else if (gamepad1.dpad_left == true){
                robot.motorLF.setPower(-1);
                robot.motorRF.setPower(1);
                robot.motorLR.setPower(1);
                robot.motorRR.setPower(-1);

            } else if (gamepad1.dpad_up == true ) {
                robot.motorLF.setPower(-1);
                robot.motorRF.setPower(-1);
                robot.motorLR.setPower(-1);
                robot.motorRR.setPower(-1);

            } else if (gamepad1.dpad_down == true) {
                robot.motorLF.setPower(1);
                robot.motorRF.setPower(1);
                robot.motorLR.setPower(1);
                robot.motorRR.setPower(1);

            } else {                //  if none of the joy stick are being manipulated, shut the motors off
                robot.motorLF.setPower(0);
                robot.motorRF.setPower(0);
                robot.motorLR.setPower(0);
                robot.motorRR.setPower(0);

            }

            if (gamepad2.y == true) {
                robot.servoFeeder.setPosition(0);
            }

            if (gamepad2.x == true) {
                robot.motorFeeder.setPower(0);
            }  else if (gamepad2.a == true) {
                robot.motorFeeder.setPower(.4);
            }

/*            telemetry.addData("left_stick_x", String.valueOf(gamepad1.left_stick_x));
            telemetry.addData("left_stick_y", String.valueOf(gamepad1.left_stick_y));
            telemetry.addData("right_stick_x", String.valueOf(gamepad1.right_stick_x));
            telemetry.addData("LF", String.valueOf(v1));
            telemetry.addData("RF", String.valueOf(v2));
            telemetry.addData("LR", String.valueOf(v3));
            telemetry.addData("RR", String.valueOf(v4));
            telemetry.update();
*/

        }

    }

    private void begin() {

        /**
         * Inititialize the robot's hardware.  The hardware configuration can be found in the
         * HardwareTestPlatform.java class.
         */
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private void shootAndFeed(){

        robot.motorShooter.setPower(1);
        while(robot.touchSensor.isPressed() == true){
            // sit and wait
        }
        robot.motorShooter.setPower(.35);
        while (robot.touchSensor.isPressed() == false){
            // sit and wait
        }

        //stop when shooter arm is reset
        robot.motorShooter.setPower(0);

        // Feed next Ball
        robot.servoFeeder.setPosition(.2);
        sleep(00);
        robot.servoFeeder.setPosition(.5);
        sleep(50);

    }

    private void shoot(){
        robot.motorShooter.setPower(.25);
        robot.motorLF.setPower(0);
        robot.motorRF.setPower(0);
        robot.motorLR.setPower(0);
        robot.motorRR.setPower(0);

        while(robot.touchSensor.isPressed() == true){
            // sit and wait
        }
        while (robot.touchSensor.isPressed() == false){
            // sit and wait
        }

        //stop when shooter arm is reset
        robot.motorShooter.setPower(0);
    }

}

