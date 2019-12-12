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
 * It raises and lowers the claw using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "PTeleOp", group = "Comp")

public class PTeleOp extends LinearOpMode {
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
    private double rightY = 0;
    private double powerLevel = 0.5;

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

            /**
             *
             * Driving algorithm
             * Note: this algorithm assumes that all values are zero when controls are not touched
             *
             **/


            /** Chassis control is for jameson meaning it should all be the way jameson wants it **/
            /** Mechanism control is for Julian therefore controls should be to his preference **/


            /**
             *
             * Code for controlling the foundation grabber
             *
             * **/

            if (gamepad2.dpad_down){
                robot.servoFoundation1.setPower(1);
                robot.servoFoundation2.setPower(.6);
            } if (gamepad2.dpad_up){
                robot.servoFoundation1.setPower(0.6);
                robot.servoFoundation2.setPower(1);
            }

            /**
             *
             * Code to manually control linear leaning mechanism
             *
             * **/

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
             * **/

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
             *
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

            /** Code to control Intake **/
            /**
             if (gamepad1.right_bumper == true){
             robot.motorIntake.setPower(0.5);
             }
             else if (gamepad1.left_bumper == true){
             robot.motorIntake.setPower(-0.5);
             }
             **/

            /** Code to control grab mechanism **/
            /**

             if (gamepad1.a == true){
             robot.motorGrab.setPower(0.5);
             }
             else if (gamepad1.b == true){
             robot.motorGrab.setPower(-0.5);
             }
             else {
             robot.motorGrab.setPower(0);
             }
             **/



            /** Intake Flip Mechanism **/
            /**
             if (gamepad1.dpad_left == true){
             robot.motorIntakeFlip.setPower(0.5);
             }
             else if (gamepad1.dpad_right == true){
             robot.motorIntakeFlip.setPower(-0.5);
             }
             else{
             robot.motorIntakeFlip.setPower(0);
             }
             **/

            idle();
 /*           telemetry.addData("left_stick_x", String.valueOf(gamepad1.left_stick_x));
            telemetry.addData("left_stick/
             if (gamepad1.left_stick_y == 0) {_y", String.valueOf(gamepad1.left_stick_y));
            telemetry.addData("right_stick_x", String.valueOf(gamepad1.right_stick_x));
            telemetry.addData("LF", String.valueOf(v1));
            telemetry.addData("RF", String.valueOf(v2));
            telemetry.addData("LR", String.valueOf(v3));
            telemetry.addData("RR", String.valueOf(v4));
            telemetry.addData("Touch Lift Forward", robot.touchLiftForward.getValue());
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