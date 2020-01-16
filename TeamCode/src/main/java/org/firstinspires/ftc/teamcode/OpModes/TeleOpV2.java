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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;

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

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Comp grabBot V2", group = "Comp")
@Disabled
public class TeleOpV2 extends LinearOpMode {
    /**
     * Instantiate all objects needed in this class
     */
    private final static HardwareProfile robot = new HardwareProfile();

    @Override
    public void runOpMode() {

        double v1;
        double v2;
        double v3;
        double v4;
        double r;
        double strafeControl;
        double robotAngle;
        double rightX;
        double powerLevel = 0.5;
        double max;

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
        telemetry.addData("COMP grabBot V2 Active", "");    //
        telemetry.update();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*
             * Driving algorithm
             * Note: this algorithm assumes that all values are zero when controls are not touched
             */
            //Calculate the power needed for each motor
//            fwdControl = -1 * gamepad1.left_stick_y;
            robotAngle = Math.atan2(gamepad1.left_stick_y, (gamepad1.left_stick_x * -1)) - Math.PI / 4;
            rightX = gamepad1.right_stick_x;
         //   rightY = gamepad1.right_stick_y;
            r = Math.hypot((gamepad1.left_stick_x * -1), gamepad1.left_stick_y);
            v1 = (r * Math.cos(robotAngle) + rightX) * powerLevel;
            v2 = (r * Math.sin(robotAngle) - rightX) * powerLevel;
            v3 = (r * Math.sin(robotAngle) + rightX) * powerLevel;
            v4 = (r * Math.cos(robotAngle) - rightX) * powerLevel;

            max = Math.max(Math.max(Math.max(Math.abs(v1), Math.abs(v2)),Math.abs(v3)),Math.abs(v4));

            if (max > 1) {
                v1 = v1/max;
                v2 = v2/max;
                v3 = v3/max;
                v4 = v4/max;
            }
            robot.motorLF.setPower(v1);
            robot.motorRF.setPower(v2);
            robot.motorLR.setPower(v3);
            robot.motorRR.setPower(v4);

            /*
             * Code for controlling the foundation grabber
             */
            if (gamepad1.x){
//                robot.servoRightGrab.setPosition(.9);
            } else if(gamepad1.y){
//                robot.servoRightGrab.setPosition(.4);
            }

            /*
             * Code to manually control lift mechanism leaning
             */

            if (gamepad1.left_trigger >0){
                robot.motorLinear.setPower(gamepad1.left_trigger);
            } else if(gamepad1.right_trigger > 0){
                robot.motorLinear.setPower(-gamepad1.right_trigger);
            }else {
                robot.motorLinear.setPower(0);
            }
            /*
             * Algorithm for placing the capstone on the foundation - temporary proceedure
             */
            if (gamepad1.right_bumper){
                robot.motorLift.setPower(-.2);
                sleep(750);
                robot.motorLift.setPower(0);
                sleep(250);
                robot.motorLift.setPower(0.2);
                sleep(750);
                robot.motorLift.setPower(0);
            }

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

