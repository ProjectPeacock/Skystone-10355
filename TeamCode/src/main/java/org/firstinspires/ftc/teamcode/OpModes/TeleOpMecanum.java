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

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Test TeleOpMecanum", group = "Test")

public class TeleOpMecanum extends LinearOpMode {
    /**
     * Instantiate all objects needed in this class
     */
    private final static HardwareProfile robot = new HardwareProfile();
    private double v1 = 0;
    private double v2 = 0;
    private double v3 = 0;
    private double v4 = 0;
    private double r = 0;
    private double fwdControl =0;
    private double strafeControl = 0;
    private double robotAngle =0;
    private double rightX = 0;

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




            if (gamepad1.left_stick_y == -1 && gamepad1.right_stick_x == 0) {//Drive Forwards (FAST)
                robot.motorLF.setPower(-0.6);
                robot.motorLR.setPower(-0.6);
                robot.motorRF.setPower(-0.6);
                robot.motorRR.setPower(-0.6);
            }//Drive Forwards (FAST)//

            else if (gamepad1.left_stick_y <= -0.3 && gamepad1.right_stick_x == 0){
                robot.motorLF.setPower(-0.2);
                robot.motorLR.setPower(-0.2);
                robot.motorRF.setPower(-0.2);
                robot.motorRR.setPower(-0.2);
            }//Drive Forwards (SLOW)//

            else if (gamepad1.left_stick_y == 1 && gamepad1.right_stick_x == 0) { //Drive backwards (FAST)
                robot.motorLF.setPower(0.6);
                robot.motorRF.setPower(0.6);
                robot.motorLR.setPower(0.6);
                robot.motorRR.setPower(0.6);
            } //Drive Backwards (FAST)//

            else if (gamepad1.left_stick_y >= 0.3 && gamepad1.right_stick_x == 0){//Drive Backwards (SLOW)
                robot.motorLF.setPower(0.2);
                robot.motorLR.setPower(0.2);
                robot.motorRF.setPower(0.2);
                robot.motorRR.setPower(0.2);
            }//Drive Backwards (SLOW)//

            else if (gamepad1.left_stick_y >= 0.3 && gamepad1.right_stick_x <= -0.3) { //Drift right (REVERSE)
                robot.motorLF.setPower(-0.6);
                robot.motorLR.setPower(-1);
                robot.motorRR.setPower(-0.5);
                robot.motorRF.setPower(-0.5);
            } //Drift right (REVERSE)

            else if (gamepad1.left_stick_y <= 0.3 && gamepad1.right_stick_x <= -0.3) { //Drift right (FORWARDS)
                robot.motorLF.setPower(1);
                robot.motorLR.setPower(0.6);
                robot.motorRR.setPower(0.5);
                robot.motorRF.setPower(0.5);
            } //Drift right(FORWARDS)

            else if (gamepad1.left_stick_y >= 0.3 && gamepad1.right_stick_x >= 0.3) { //Drift left (REVERSE)
                robot.motorLF.setPower(-0.5);
                robot.motorLR.setPower(-0.5);
                robot.motorRR.setPower(-1);
                robot.motorRF.setPower(-0.6);
            } //Drift left (REVERSE)

            else if (gamepad1.left_stick_y <= -0.3 && gamepad1.right_stick_x >= 0.3) { //Drift left (FORWARDS)
                robot.motorLF.setPower(0.5);
                robot.motorLR.setPower(0.5);
                robot.motorRR.setPower(0.6);
                robot.motorRF.setPower(1);
            }//Drift left (FORWARDS)

            else if (gamepad1.right_stick_x <= -0.3 && gamepad1.left_stick_y == 0){ //Turn Right
                robot.motorRF.setPower(1);
                robot.motorRR.setPower(1);
                robot.motorLF.setPower(-1);
                robot.motorLR.setPower(-1);
              } //Turn Right//

            else if (gamepad1.right_stick_x >= 0.3 && gamepad1.left_stick_y == 0){
                robot.motorRF.setPower(-1);
                robot.motorRR.setPower(-1);
                robot.motorLF.setPower(1);
                robot.motorLR.setPower(1);
            }//Turn left

            else if (gamepad1.left_stick_x <= -0.3 && gamepad1.right_stick_x == 0){ //Strafe right
                robot.motorRF.setPower(-1);
                robot.motorRR.setPower(1);
                robot.motorLF.setPower(0.7);
                robot.motorLR.setPower(-0.7);
            }//Strafe right//

            else if (gamepad1.left_stick_x >= 0.3 && gamepad1.right_stick_x == 0){ //Strafe left
                robot.motorRF.setPower(0.7);
                robot.motorRR.setPower(-0.7);
                robot.motorLF.setPower(-1);
                robot.motorLR.setPower(1);
            }//Strafe left//

            else if (gamepad1.left_stick_y >= -0.3 && gamepad1.right_stick_y <= -0.3) { //Drive forwards full speed
                robot.motorLF.setPower(1);
                robot.motorLR.setPower(1);
                robot.motorRR.setPower(1);
                robot.motorRF.setPower(1);

            }//Drive forwards full speed

            else if (gamepad1.left_stick_y <= -0.3 && gamepad1.right_stick_y >= -0.3) { //Drive backwards full speed
                robot.motorLF.setPower(-1);
                robot.motorLR.setPower(-1);
                robot.motorRR.setPower(-1);
                robot.motorRF.setPower(-1);

            }//Drive backwards full speed

            else {
                robot.motorLF.setPower(0);
                robot.motorLR.setPower(0);
                robot.motorRF.setPower(0);
                robot.motorRR.setPower(0);
            }//Full Stop//

            //ATTACHMENTS

             if (gamepad1.right_trigger > (0)) {//4-Bar Lift
                 robot.motorLift.setPower(-gamepad1.right_trigger);
             }//4-Bar Lift

             else if (gamepad1.left_trigger > (0)) {//4-Bar Lower
                 robot.motor4Bar.setPower(gamepad1.left_trigger);
             }//4-Bar Lower
                else {
                 robot.motor4Bar.setPower(0);
             }//Full Stop

            idle();
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

}

