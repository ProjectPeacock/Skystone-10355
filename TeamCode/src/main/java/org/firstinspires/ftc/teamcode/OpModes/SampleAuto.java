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

/*
    Team:       10355 - Project Peacock
    Autonomous Program - Red Strategy #1
    Alliance Color: Red
    Robot Starting Position: Red zone, wall near ramp
    Strategy Description:
        - Press correct button on beacon near ramp
        - Press correct button on beacon furthest from ramp
        - Park a wheel on the red ramp

    Hardware Setup:
        - 4 mecanum wheels with encoder on LF wheel - encoder utilized for measuring distance for fwd/rev drive operation
        - Arm Motor with encoder - controls mechanism for dumping particles into ramp
        - Gyro sensor located at the center of the robot - utilized to compensate for drift
        - 1 x Color sensor (colorSensorLeft)- utilized to identify beacon color
        - 1 x Touch sensor - utilized to identify when robot touches wall with the front of the robot
        - 1 x Optical Distance Sensor (ODS) - utilized to locate the white lines on the floor
        - 1 x Motorola Camera - Utilized for Vuforia positioning of the robot on the field

    State Order:
        - ACQUIRE_RED_BEACON_LEFT       // moves from the wall to the first beacon closest to the ramp
        - PUSH_BUTTON_RED               // Identifies which button to press, right or left
        - PRESS_BUTTON                  // presses the correct button - also validates that the button is pressed
                                        // and attempts to press over again until the button is pressed
        - ACQUIRE_RED_BEACON_RIGHT      // moves from the wall to the second beacon on the right of the field
        - PUSH_BUTTON_RED               // Identifies which button to press, right or left
        - PRESS_BUTTON                  // presses the correct button - also validates that the button is pressed
                                        // and attempts to press over again until the button is pressed
        - END_GAME                      // identifies the last actions before the end of autonomous mode
        - RAMP                          // For this strategy, the robot will end with a wheel parked on the ramp
        - HALT                          // Shutdown sequence for autonomous mode

 */
package org.firstinspires.ftc.teamcode.OpModes;

/**
 * Import the classes we need to have local access to.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.Libs.DataLogger;
import org.firstinspires.ftc.teamcode.Libs.VuforiaLib;
import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;

import java.util.List;

/**
 * Name the opMode and put it in the appropriate group
 */
@Autonomous(name = "Red 1 - Beacons, Ramp", group = "COMP")

public class SampleAuto extends LinearOpMode {

    /**
     * Instantiate all objects needed in this class
     */

    private final static HardwareProfile robot = new HardwareProfile();
    double radians = 0;
    private VuforiaLib myVuforia = new VuforiaLib();
    private ElapsedTime runtime = new ElapsedTime();
    /**
     * Define global variables
     */
    private double mm = 500;            //Distance for translateDistance method
    private double power = .6;          //Motor power for all methods
    private double heading = 90;        //Heading for all methods
    private double y = -200;            //Vuforia y stop coordinate
    private double x = -200;            //Vuforia x stop coordinate
    private double changeSpeed = 0;     //Rotation speed for motor speed calculations
    private double initZ;               //The initial reading from integrated Z
    private double currentZint;         //Current integrated Z value
    private double zCorrection;         //Course correction in degrees
    private double timeOut = 5;         //Timeout in seconds for translateTime method
    private double timeOutTime;         //Calculated time to stop
    private String procedure;           //Name of the method that is running
    private double odsThreshold = .3;   //Threshold at which the ODS sensor acquires the whie line
    private double ods = 0;             //Value returned from the Optical Distance Sensor
    private double colorRightRed = 0;   //Value from color sensor
    private double colorRightBlue = 0;  //Value from color sensor
    private double colorLeftRed = 0;    //Value from color sensor
    private double colorLeftBlue = 0;   //Value from color sensor
    private double robotX;              // The robot's X position from VuforiaLib
    private double robotY;              // The robot's Y position from VuforiaLib
    private double robotBearing;        //Bearing to, i.e. the bearing you need to steer toward
    private double LF, RF, LR, RR;      //Motor power setting
    private double myCurrentMotorPosition;  //Current encoder position
    private double myTargetPosition;        //Target encoder position
    private boolean colorLedEnable = false; //Enable if in testing mode, disable for beacon
    private boolean leftAlign = false;      //
    private boolean beaconFlag = false;
    private List<Double> vuforiaTracking;   //List of Vuforia coordinates
    private List<VuforiaTrackable> myTrackables;    //List of Vuforia trackable objects
    private DataLogger Dl;                          //Datalogger object
    private double motorCorrectCoefficient = .05;    //Amount to divide the zInt drift by
    private String beaconColorRight;                //Color of the right side of the beacon
    private String beaconColorLeft;                 //Color of the left side of the beacon
    private String button;                          //Which button to push
    private String alliance = "red";                //Your current alliance
    private String beaconState = "inactive";            //Has the beacon been triggered?
    private String courseCorrect = "";
    private State state = State.ACQUIRE_RED_BEACON_LEFT;    //Machine State
    private String nextState = "Beacon2";


    public void runOpMode() {
        /**
         * Setup the init state of the robot.  This configures all the hardware that is defined in
         * the HardwareTestPlatform class.
         */
        robot.init(hardwareMap);

        /**
         * Set the initial servo positions
         */
        robot.servoInTake.setPosition(1);

        /**
         *  Create the DataLogger object.
         */
        createDl();

        /**
         * Calibrate the gyro
         */
        robot.servoInTake.setPosition(.3);
        robot.servoBallBumper.setPosition(.9);
        robot.servoFeeder.setPosition(0.6);


        robot.sensorGyro.calibrate();
        while (robot.sensorGyro.isCalibrating()) {
            telemetry.addData("Waiting on Gyro Calibration", "");
            telemetry.update();
        }

        /**
         * Initialize Vuforia and retrieve the list of trackable objects.
         */
        telemetry.addData("Waiting on Vuforia", "");
        telemetry.update();

        myTrackables = myVuforia.vuforiaInit();

        telemetry.addData("Status", "Vuforia Initialized");
        telemetry.update();
        sleep(1000);

        /**
         * Position Shooter
         */
        telemetry.addData("Status", "Initializing Shooter");
        telemetry.update();
        sleep(1000);
        robot.motorShooter.setPower(.25);
        while (robot.touchSensor.isPressed() == true) {
        }
        //stop if button is not pressed
        robot.motorShooter.setPower(0);


        telemetry.addData(">", "Place ball in shooter & Ready");
        telemetry.update();

        /**
         * Start the opMode
         */
        waitForStart();

        while (opModeIsActive()) {
            /**
             * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
             * This is the section of code you should change for your robot.
             * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
             */
            // FOO
            switch (state) {
                case ACQUIRE_RED_BEACON_LEFT:
                    beaconState = "inactive";   // set beaconState to inactive <- will be reset when
                                                // the beacon is the correct color

                    heading = 55;               // set diagonal heading towards first beacon
                    power = 1;                  // apply full power to the drive wheels
                    timeOut = 3.5;              // 2 1/2 seconds is the time required from wall
                                                // positioning
                    translateTime();            // apply drive using settings

                    heading = 90;               // strafe towards white line for first beacon
                    power = 1;                  // apply full power to the drive wheels
                    timeOut = .4;               // time required from wall positioning
                    translateTime();            // apply drive using settings

                    heading = 90;               // Continue to strafe towards white line for first
                                                // beacon
                    power = .2;                 // Reduce power to allow for acquisition of white
                                                // line with ODS
                    translateOdsStop();         // continues to strafe until the white line is
                                                // located

                    sleep(500);                // Pause for 1 second once the white line is located
                                                // to allow
                                                // for the Vuforia to acquire positioning data
                    x = -1420;                  // Set Vuforia positioning
                    heading = 0;                // set bearing towards the beacon
                    acquireBeaconX();           // drive forward to beacon

//                    sleep(1000);                // pause for 1 second
                    // set the next state to push the red button
                    state = State.PUSH_BUTTON_RED;
                    //Exit the state
                    break;

                case ACQUIRE_RED_BEACON_RIGHT:
                    beaconState = "inactive";   // set beaconState to inactive <- will be reset when
                                                // the beacon is the correct color

                    heading = 180;              // back away from the beacon
                    power = 1;                  // apply full power
                    timeOut = .30;              // backup for 1/2 second
                    translateTime();            // apply parameters

                    heading = 90;               // strafe towards the second beacon
                    power = 1;                  // apply full power
                    timeOut = 2.5;              // strafe for 2 seconds
                    translateTime();            // apply parameters

                    power = .3;                 // reduce power to increase ODS acquisition of white
                                                // line
                    heading = 90;               // set course towards white line
                    translateOdsStop();         // continue strafing until the white line is
                                                // discovered

                    sleep(500);                // Pause for 1 second once the white line is located

                    x = -1420;                  // set position for stopping the robot
                    power = .2;                  // apply full power
                    heading = 0;                // set heading to drive forward
                    acquireBeaconX();           // move forward into position

                    nextState = "end_game";     // tells the program to go to the end game when done
                                                // pressing buttons
                    state = State.PUSH_BUTTON_RED;     // press the correct button
                    break;

                case PUSH_BUTTON_RED:
                    getColor();                 // will set variable "color" equal to red or blue

                    telemetry();                // print color to screen
                    getButtonPush();            // calculates which button to press and sets
                                                // variable "button" to left or right
                    telemetry();                // print button info to screen

                    //
                    state = State.PRESS_BUTTON;     // move over to the next beacon
                    break;

                case PRESS_BUTTON:              // State to press the button once aligned => will

                    pushRedButton_10355();      // method for pressing the beacon button

                    if (nextState == "Beacon2") {
                        state = State.ACQUIRE_RED_BEACON_RIGHT;     // move over to the next beacon
                    } else if (nextState == "end_game"){
                        state = State.END_GAME;                     // go to the end game state
                    }
                    telemetry();

                    break;

                case END_GAME:
                    heading = 180;              // strafe towards the second beacon
                    power = 1;                  // apply full power
                    timeOut = .75;              // strafe for 2 seconds
                    translateTime();            // apply parameters

                    state = State.RAMP;
                    break;

                case RAMP:                      // head to ramp and park on the ramp
                    heading = 45;              // turn towards the ramp
                    pivot();

                    heading = 180;                // drive forward
                    power = 1;                  // apply full power
                    timeOut = 4;                // time required to get to the ramp
                    translateTime();            // apply parameters

                    state = state.HALT;

                    break;

                case CAP:                      // head to ramp and park on the ramp
                    heading = 20;              // turn towards the ramp
                    pivot();

                    heading = 180;              // drive backward
                    power = 1;                  // apply full power
                    timeOut = 2;                // time required to get to the ramp
                    translateTime();            // apply parameters

                    state = state.HALT;

                    break;

                case SHOOTER:                   // Shoot balls into the upper basket
                    state = State.HALT;
                    break;

                case HALT:
                    motorsHalt();               //Stop the motors

                    //Stop the DataLogger
                    dlStop();

                    //Exit the OpMode
                    requestOpModeStop();
                    break;
            }
            /**
             * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
             * Don't change anything past this point.  Bad things could happen.
             * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
             */
        }
    }

    /**
     * Transmit telemetry.
     */
    private void telemetry() {

        telemetry.addData("Alliance", String.valueOf(alliance));
        telemetry.addData("State", String.valueOf(state));
        telemetry.addData("Procedure", String.valueOf(procedure));
        telemetry.addData("beaconColorRight", String.valueOf(beaconColorRight));
        telemetry.addData("beaconColorLeft", String.valueOf(beaconColorLeft));
        telemetry.addData("beaconState", String.valueOf(beaconState));
        telemetry.addData("button", String.valueOf(button));
        telemetry.addData("Heading", String.valueOf(heading));
        telemetry.addData("robotX", String.valueOf((int) robotX));
        telemetry.addData("robotY", String.valueOf((int) robotY));
        telemetry.addData("Target X", String.valueOf(x));
        telemetry.addData("Target Y", String.valueOf(y));
        telemetry.addData("robotBearing", String.valueOf((int) robotBearing));
        telemetry.addData("Current Z Int", String.valueOf(currentZint));
        telemetry.addData("Z Correction", String.valueOf(zCorrection));
        telemetry.addData("touchSensor", String.valueOf(robot.touchSensor.getValue()));
        telemetry.addData("ODS", String.valueOf(ods));
        telemetry.addData("Color Right Red", String.valueOf(colorRightRed));
        telemetry.addData("Color Right Blue", String.valueOf(colorRightBlue));
        telemetry.addData("Color Left Red", String.valueOf(colorLeftRed));
        telemetry.addData("Color Left Blue", String.valueOf(colorLeftBlue));
        telemetry.addData("Target Encoder Position", String.valueOf(myTargetPosition));
        telemetry.addData("Current Encoder Position", String.valueOf(robot.motorLF.getCurrentPosition()));
        telemetry.addData("LF", String.valueOf(LF));
        telemetry.addData("RF", String.valueOf(RF));
        telemetry.addData("LR", String.valueOf(LR));
        telemetry.addData("RR", String.valueOf(RR));
        telemetry.update();
    }

    private void pushRedButton_10355() {
        // method updated for 10355

        while (beaconState == "inactive"){
            if (button.equals("right")) {
                // drive forward into the beacon to press the button
                power = 0.3;
                heading = 0;
                timeOut = 0.5;
                translateTime();

            }

            if (button.equals("left")) {
                if (leftAlign == false) {

                        // to press the left button, the robot needs to strafe to the right
                        heading = 90;               // strafe towards white line for first beacon
                        power = .6;                 // reduce power to the drive wheels
                        timeOut = .7;               // Time needs to be optimized...
                        translateTime();            // apply drive using settings

                        leftAlign = true;
                }


                // drive forward into the beacon to press the button
                power = 0.6;
                heading = 0;
                timeOut = 0.3;
                translateTime();

                // drive backward from the beacon to press the button
//                power = 0.6;
//                heading = 180;
//                timeOut = 0.3;
//                translateTime();

            }
            sleep(500);                // allow the beacon light to stableize
            checkBeacon_10355();

            power = 0.2;
            heading = 180;
            timeOut = 0.3;
            translateTime();


            if(beaconState == "activated")
            {
                leftAlign = false;
                break;
            }
            telemetry.update();

        }
    }

    private void getButtonPush() {
        if (beaconColorRight.equals("red") && alliance.equals("red")) {
            button = "left";
        }

        if (beaconColorRight.equals("blue") && alliance.equals("red")) {
            button = "right";
        }

        if (beaconColorRight.equals("red") && alliance.equals("blue")) {
            button = "right";
        }

        if (beaconColorRight.equals("blue") && alliance.equals("blue")) {
            button = "left";
        }

    }

    private void checkBeacon_10355() {

        if (colorLedEnable) {
            robot.colorSensorRight.enableLed(true);
        } else {
            robot.colorSensorRight.enableLed(false);
        }

        if (alliance.equals("red")) {
            if (robot.colorSensorRight.red() >= 2) {  //Beacon activated
                beaconState = "activated";
            } else {
                beaconState = "inactive";
            }
        }

        if (alliance.equals("blue")) {
            if (robot.colorSensorRight.blue() >= 2) {  //Beacon activated
                beaconState = "activated";
            } else {
                beaconState = "inactive";
            }
        }
    }

    private void getColor() {
        String color = "unk";
        double red;
        double blue;

        if (colorLedEnable) {
            robot.colorSensorRight.enableLed(true);
        } else {
            robot.colorSensorRight.enableLed(false);
        }

        sleep(100);

        while (color.equals("unk")) {
            red = robot.colorSensorRight.red();
            blue = robot.colorSensorRight.blue();

            if (red >= 2) {
                color = "red";
            }
            if (blue >= 2) {
                color = "blue";
            }
            beaconColorRight = color;

            idle();
        }
    }

    /**
     * Pivot the robot to a new heading. 0 is straight ahead, 1 to 179 is to the left -1 to -179 is
     * to the right.
     */
    private void pivot() {
        procedure = "Pivot";
        initZ = robot.mrGyro.getIntegratedZValue();

        if (heading > 0) {
            while (currentZint < heading && opModeIsActive()) {
                /**
                 * We are pivoting left so reverse power to the left motor
                 */
                LR = -power;
                LF = -power;
                RR = power;
                RF = power;

                setPower();

                getSensorData();

                logData();
                idle();
            }
        }
        if (heading < 0) {
            while (currentZint > heading && opModeIsActive()) {
                /**
                 * We are pivoting right so reverse power to the right motor
                 */
                LR = power;
                LF = power;
                RR = -power;
                RF = -power;

                setPower();

                getSensorData();

                logData();
                idle();
            }
        }
        motorsHalt();
    }

    /**
     * Catchall to get data from all sensor systems.  Updates global variables
     */
    private void getSensorData() {
        colorRightRed = robot.colorSensorRight.red();
        colorRightBlue = robot.colorSensorRight.blue();
        colorLeftRed = robot.colorSensorLeft.red();
        colorLeftBlue = robot.colorSensorLeft.blue();
        ods = robot.ods.getLightDetected();
        currentZint = robot.mrGyro.getIntegratedZValue();
        vuforiaTracking = myVuforia.getLocation(myTrackables);
        robotX = vuforiaTracking.get(0);
        robotY = vuforiaTracking.get(1);
        robotBearing = vuforiaTracking.get(2);
    }

    /**
     * In our testing, on our field, a .7 value indicates you are centered on
     * the line.  You should use the practice field and test opMode to confirm
     * this is valid for your actual competition field.
     * The value from the ODS sensor indicates you are no longer centered on the
     * line and the robotBearing indicates the bot is to the left of the line
     * so we will increase power on the LR and decrease power on the RR to
     * get centered back on the line
     */
    private void followLineX() {
        procedure = "followLineX";
        initZ = robot.mrGyro.getIntegratedZValue();

        while (robotX > x && opModeIsActive()) {

            getSensorData();

            LF = calcLF();
            RF = calcRF();
            LR = calcLR();
            RR = calcRR();

            currentZint = robot.mrGyro.getIntegratedZValue();

            if (ods < .5 && robotBearing > 0 && robotBearing < 180) {
                LF = power + .05;
                LR = power + .05;
                RF = power - .05;
                RR = power - .05;

                telemetry.addData("DRIFTED LEFT", String.valueOf(robotBearing));
            }

            /**
             * The robot has drifted right, correct the course
             */
            if (ods < .5 && robotBearing > 180 && robotBearing < 359.9) {
                LF = power - .05;
                LR = power - .05;
                RF = power + .05;
                RR = power + .05;

                telemetry.addData("DRIFTED RIGHT", "");
            }

            /**
             * The robot is centered on the line, drive straight.
             */
            if (ods > .5) {
                LF = calcLF();
                RF = calcRF();
                LR = calcLR();
                RR = calcRR();

                telemetry.addData("NO DRIFT", "");
            }

            setPower();

            logData();
            idle();
        }
        motorsHalt();
    }

    private void acquireBeaconX() {
        procedure = "acquireBeaconX";
        initZ = robot.mrGyro.getIntegratedZValue();

        getSensorData();

        while (robotX > x && opModeIsActive()) {

            getSensorData();

            LF = calcLF();
            RF = calcRF();
            LR = calcLR();
            RR = calcRR();

            currentZint = robot.mrGyro.getIntegratedZValue();

            if (ods < .9 && robotBearing > 0 && robotBearing < 180) {
                LF = power + power * .1;
                LR = power + power * .1;
                RF = power + power * .1;
                RR = power + power * .1;

                telemetry.addData("DRIFTED LEFT", String.valueOf(robotBearing));
            }

            /**
             * The robot has drifted right, correct the course
             */
            if (ods < .9 && robotBearing > 180 && robotBearing < 359.9) {
                LF = power + power * .1;
                LR = power + power * .1;
                RF = power + power * .1;
                RR = power + power * .1;

                telemetry.addData("DRIFTED RIGHT", "");
            }

            /**
             * The robot is centered on the line, drive straight.
             */
            if (ods > .9) {
                LF = power;
                RF = power;
                LR = power;
                RR = power;

                telemetry.addData("NO DRIFT", "");
            }

            setPower();

            logData();
            idle();
        }
        motorsHalt();
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
        Dl.addField("Procedure");
        Dl.addField("courseCorrect");
        Dl.addField("heading");
        Dl.addField("robotX");
        Dl.addField("robotY");
        Dl.addField("X");
        Dl.addField("Y");
        Dl.addField("robotBearing");
        Dl.addField("initZ");
        Dl.addField("currentZ");
        Dl.addField("zCorrection");
        Dl.addField("touchSensor");
        Dl.addField("ODS");
        Dl.addField("colorRightRed");
        Dl.addField("colorRightBlue");
        Dl.addField("colorLeftRed");
        Dl.addField("colorLeftBlue");
        Dl.addField("LFTargetPos");
        Dl.addField("LFMotorPos");
        Dl.addField("LF");
        Dl.addField("RF");
        Dl.addField("LR");
        Dl.addField("RR");
        Dl.newLine();
    }

    /**
     * Log data to the file on the phone.
     */
    private void logData() {

        Dl.addField(String.valueOf(runtime.time()));
        Dl.addField(String.valueOf(alliance));
        Dl.addField(String.valueOf(state));
        Dl.addField(String.valueOf(procedure));
        Dl.addField(String.valueOf(courseCorrect));
        Dl.addField(String.valueOf(heading));
        Dl.addField(String.valueOf((int) robotX));
        Dl.addField(String.valueOf((int) robotY));
        Dl.addField(String.valueOf(x));
        Dl.addField(String.valueOf(y));
        Dl.addField(String.valueOf((int) robotBearing));
        Dl.addField(String.valueOf(initZ));
        Dl.addField(String.valueOf(currentZint));
        Dl.addField(String.valueOf(zCorrection));
        Dl.addField(String.valueOf(robot.touchSensor.getValue()));
        Dl.addField(String.valueOf(ods));
        Dl.addField(String.valueOf(colorRightRed));
        Dl.addField(String.valueOf(colorRightBlue));
        Dl.addField(String.valueOf(colorLeftRed));
        Dl.addField(String.valueOf(colorLeftBlue));
        Dl.addField(String.valueOf(myTargetPosition));
        Dl.addField(String.valueOf(robot.motorLF.getCurrentPosition()));
        Dl.addField(String.valueOf(LF));
        Dl.addField(String.valueOf(RF));
        Dl.addField(String.valueOf(LR));
        Dl.addField(String.valueOf(RR));
        Dl.newLine();
    }

    /**
     * Stop the DataLogger
     */
    private void dlStop() {
        Dl.closeDataLogger();

    }

    /**
     * Translate on a heading and stop when the ODS sensor detects the white line.
     */
    private void translateOdsStop() {
        procedure = "translateOdsStop";
        initZ = robot.mrGyro.getIntegratedZValue();

        getSensorData();

        radians = heading * (Math.PI / 180);

        while (opModeIsActive() && ods < odsThreshold) {
            ods = robot.ods.getLightDetected();

            LF = calcLF();
            RF = calcRF();
            LR = calcLR();
            RR = calcRR();

            currentZint = robot.mrGyro.getIntegratedZValue();

            if (currentZint != initZ) {  //Robot has drifted off course
                zCorrection = Math.abs(initZ - currentZint);

                courseCorrect();
            } else {
                zCorrection = 0;
            }

            setPower();

            myCurrentMotorPosition = robot.motorLF.getCurrentPosition();

            getSensorData();

            //telemetry();
            //logData();

            idle();
        }

        motorsHalt();

    }

    /**
     * Translate on a heading the distance specified in MM.
     */
    private void translateDistance() {
        procedure = "translateDistance";
        initZ = robot.mrGyro.getIntegratedZValue();
        myCurrentMotorPosition = robot.motorLF.getCurrentPosition();
        myTargetPosition = myCurrentMotorPosition + (int) (mm * robot.COUNTS_PER_MM);

        getSensorData();

        radians = heading * (Math.PI / 180);

        while (opModeIsActive() && myTargetPosition > myCurrentMotorPosition) {
            LF = calcLF();
            RF = calcRF();
            LR = calcLR();
            RR = calcRR();

            currentZint = robot.mrGyro.getIntegratedZValue();

            if (currentZint != initZ) {  //Robot has drifted off course
                zCorrection = Math.abs(initZ - currentZint);

                courseCorrect();
            } else {
                zCorrection = 0;
            }

            setPower();

            myCurrentMotorPosition = robot.motorLF.getCurrentPosition();

            getSensorData();

            telemetry();
            logData();

            idle();
        }

        motorsHalt();

    }


    /**
     * Translate on a heading for a defined period of time.
     */
    private void translateTime() {
        procedure = "translateTime";
        initZ = robot.mrGyro.getIntegratedZValue();
        getSensorData();

        radians = heading * (Math.PI / 180);
        timeOutTime = runtime.time() + timeOut;

        while (opModeIsActive() && runtime.time() < timeOutTime) {
            LF = calcLF();
            RF = calcRF();
            LR = calcLR();
            RR = calcRR();

            currentZint = robot.mrGyro.getIntegratedZValue();

            if (currentZint != initZ) {  //Robot has drifted off course
                zCorrection = Math.abs(initZ - currentZint);

                courseCorrect();
            } else {
                zCorrection = 0;
            }

            setPower();

            myCurrentMotorPosition = robot.motorLF.getCurrentPosition();

            getSensorData();

            telemetry();
            logData();

            idle();
        }

        motorsHalt();

    }

    /**
     * Cut power to the motors.
     */
    private void motorsHalt() {
        robot.motorLF.setPower(0);
        robot.motorRF.setPower(0);
        robot.motorLR.setPower(0);
        robot.motorRR.setPower(0);
    }

    /**
     * Translate on a heading and stop when we reach the specified X axis value.
     */
    private void translateVuforiaNavXNeg() {
        procedure = "translateVuforiaNavXNeg";
        initZ = robot.mrGyro.getIntegratedZValue();
        radians = heading * (Math.PI / 180);

        getSensorData();

        while (opModeIsActive() && robotX > x) {
            LF = calcLF();
            RF = calcRF();
            LR = calcLR();
            RR = calcRR();

            currentZint = robot.mrGyro.getIntegratedZValue();

            if (currentZint != initZ) {  //Robot has drifted off course
                zCorrection = Math.abs(initZ - currentZint);

                courseCorrect();
            } else {
                zCorrection = 0;
            }

            setPower();

            myCurrentMotorPosition = robot.motorLF.getCurrentPosition();

            getSensorData();

            telemetry();
            logData();

            idle();
        }

        motorsHalt();

    }


    /**
     * Translate on a heading and stop when we reach the specified X axis value.
     */
    private void translateVuforiaNavXPos() {
        procedure = "translateVuforiaNavXNeg";
        initZ = robot.mrGyro.getIntegratedZValue();
        radians = heading * (Math.PI / 180);

        getSensorData();

        while (opModeIsActive() && robotX < x) {
            LF = calcLF();
            RF = calcRF();
            LR = calcLR();
            RR = calcRR();

            currentZint = robot.mrGyro.getIntegratedZValue();

            if (currentZint != initZ) {  //Robot has drifted off course
                zCorrection = Math.abs(initZ - currentZint);

                courseCorrect();
            } else {
                zCorrection = 0;
            }

            setPower();

            myCurrentMotorPosition = robot.motorLF.getCurrentPosition();

            getSensorData();

            telemetry();
            logData();

            idle();
        }

        motorsHalt();

    }

    /**
     * Translate on a heading and stop when we reach the specified Y axis value.
     */
    private void translateVuforiaNavY() {
        procedure = "translateVuforiaNavY";
        initZ = robot.mrGyro.getIntegratedZValue();

        getSensorData();

        radians = heading * (Math.PI / 180);

        while (opModeIsActive() && robotY < y) {
            LF = calcLF();
            RF = calcRF();
            LR = calcLR();
            RR = calcRR();

            currentZint = robot.mrGyro.getIntegratedZValue();

            if (currentZint != initZ) {  //Robot has drifted off course
                zCorrection = Math.abs(initZ - currentZint);

                courseCorrect();
            } else {
                zCorrection = 0;
            }

            setPower();

            myCurrentMotorPosition = robot.motorLF.getCurrentPosition();

            getSensorData();

            telemetry();
            logData();

            idle();
        }

        motorsHalt();

    }

    /**
     * Calculate the wheel speeds.
     *
     * @return wheel speed
     */
    private double calcLF() {
        LF = power * Math.sin(radians + (Math.PI / 4)) + changeSpeed;

        if (LF > 1 || LF < -1) {
            LF = 0;
        }

        return LF;
    }

    private double calcRF() {
        RF = power * Math.cos(radians + (Math.PI / 4)) - changeSpeed;

        if (RF > 1 || RF < -1) {
            RF = 0;
        }

        return RF;
    }

    private double calcLR() {
        LR = power * Math.cos(radians + (Math.PI / 4)) + changeSpeed;

        if (LR > 1 || LR < -1) {
            LR = 0;
        }

        return LR;
    }

    private double calcRR() {
        RR = power * Math.sin(radians + (Math.PI / 4)) - changeSpeed;

        if (RR > 1 || RR < -1) {
            RR = 0;
        }

        return RR;
    }

    /**
     * TODO: Test in all modes
     */
    /**
     * Change power to the motors to correct for heading drift as indicated by the gyro.
     */
    private void courseCorrect() {
        if (currentZint > initZ) {  //Robot has drifted left
            LF = LF + motorCorrectCoefficient;
            LR = LR + motorCorrectCoefficient;
            RF = RF - motorCorrectCoefficient;
            RR = RR - motorCorrectCoefficient;
        }

        if (currentZint < initZ) {  //Robot has drifted right
            LF = LF - motorCorrectCoefficient;
            LR = LR - motorCorrectCoefficient;
            RF = RF + motorCorrectCoefficient;
            RR = RR + motorCorrectCoefficient;
        }
    }

    /**
     * Set the power level of the motors.
     */
    private void setPower() {
        robot.motorLF.setPower(LF);
        robot.motorRF.setPower(RF);
        robot.motorLR.setPower(LR);
        robot.motorRR.setPower(RR);
    }

    /**
     * Enumerate the States of the machine.
     */
    enum State {
        ACQUIRE_RED_BEACON_LEFT, ACQUIRE_RED_BEACON_RIGHT, PUSH_BUTTON_RED, PUSH_BUTTON_BLUE,
        TEST, HALT, END_GAME, RAMP, PRESS_BUTTON, SHOOTER, CAP
    }

}
