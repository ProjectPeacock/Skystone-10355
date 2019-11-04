/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Libs.DataLogger;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;
import org.firstinspires.ftc.teamcode.Libs.skystoneVuforia;
import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;


import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * This 2019-2020 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the SKYSTONE FTC field.
 * The code is structured as a LinearOpMode
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.

 * Eight perimeter targets are distributed evenly around the four perimeter walls
 * Four Bridge targets are located on the bridge uprights.
 * Refer to the Field Setup manual for more specific location details
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  skystone/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */


@Autonomous(name="SKYSTONE Vuforia Test", group ="Concept")
//@Disabled
public class vuforiaTest extends LinearOpMode {

    /**
     * Instantiate all objects needed in this class
     */

    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;                     //Opmode
    double radians = 0;
    private skystoneVuforia myVuforia = new skystoneVuforia();
    private ElapsedTime runtime = new ElapsedTime();


    private State state = State.FOURTH_STATE;    //Machine State
    private double robotX;              // The robot's X position from VuforiaLib
    private double robotY;              // The robot's Y position from VuforiaLib
    private double robotBearing;        //Bearing to, i.e. the bearing you need to steer toward
    private double robotRoll;
    private DataLogger Dl;                          //Datalogger object

    private boolean rollAlign = false;
    // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
    // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
    //
    // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    //
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AQnFmuP/////AAABmQCm3/d2XkmSr01g6YE3lFSJSgwnTpe+36Ubl1lNvf5SfnVBBt2kW6Jl/wXN//IX3CfcSOldD2PTFY56tUPu0R45yCVtA3+y33VKlzKXMR1nFbYjOvD6BiG2fmDIx8ViGKvq0tr1NZQo8XpeTVL8N79dxMSHzUHBoehIzrtniGKoeaYcr4H6oGAxOXp0GLebanWq61B6VWxKp4etuwzS9OX86R+PMVAXHBTJLWpOm2WeTIeCopZ46wfpzVZDeI6BEXHN84QzFGM8g4lmTxwBizXxUE08tlOjTl+V/+EkkDsMHys+x9f/hyXCetITnAmRiiueOFzYhx5XBItX9msyLLU/TinrMIPICW7U5IAU8kdh";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;
    private float roll              = 0;
    String targetName = "";
    double visibleTarget = 99;       // identifies which target is the visible target
    private int stage = 0;

//    private double visibleTarget;

    @Override public void runOpMode() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */

        telemetry.addData("STATUS : ", "INITIALIZING SYSTEM");
        telemetry.addData("DO NOT PRESS PLAY!", "");
        telemetry.update();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));



        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        telemetry.addData("Status", "Waiting for Vuforia to initialize");
        telemetry.update();

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        /**
         * Setup the init state of the robot.  This configures all the hardware that is defined in
         * the HardwareTestPlatform class.
         */
        robot.init(hardwareMap);

        /**
         * Instantiate the drive class
         */
//        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
        DriveMecanum drive = new DriveMecanum(robot, opMode, myVuforia, targetsSkyStone);


        targetsSkyStone.activate();

//        sleep(5000);

        targetVisible = false;
        telemetry.addData("Status", "Vuforia Initialized");
        telemetry.addData(">", "System initialized and Ready");
        telemetry.update();


        waitForStart();

        runtime.reset();
        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        while (opModeIsActive()) {

            switch (state) {
                case FOUNDATION:
                    if (runtime.time() > 24){
                        state = State.END_STATE;
                        break;
                    }

                    /**
                     * Provide a description of what this state does
                     * Code goes here
                     */

                    drive.translate(.5,20, 1);
                    drive.motorsHalt();

//                    robot.servoRightGrab.setPosition(0.5);
//                    robot.servoLeftGrab.setPosition(0.5);

                    drive.translate(-1,180,.5);
                    drive.motorsHalt();
                    robot.motorRF.setPower(-0.5);
                    robot.motorRR.setPower(-0.5);
                    sleep(400);
                    drive.motorsHalt();

                    drive.translate(1, 0, .3);
                    drive.motorsHalt();

//                    robot.servoRightGrab.setPosition(0);
//                    robot.servoLeftGrab.setPosition(0);

                    state = State.SECOND_STATE;
                    //Exit the state
                    break;

                case VUFORIA_TEST:

                    if (runtime.time() > 24){
                        state = State.END_STATE;
                        break;
                    }
                    /**
                     * This state is intended to test the Vuforia code and provide results to the
                     * Driver Station screen
                     */

                    // check all the trackable targets to see which one (if any) is visible.
                    for (VuforiaTrackable trackable : allTrackables) {
                        if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                            targetName= trackable.getName();
                            telemetry.addData("Visible Target", targetName);
                            targetVisible = true;

                            // getUpdatedRobotLocation() will return null if no new information is available since
                            // the last time that call was made, or if the trackable is not currently visible.
                            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                            if (robotLocationTransform != null) {
                                lastLocation = robotLocationTransform;
                            }
                            break;
                        } else {
                            targetVisible = false;
                        }

                        visibleTarget = 99;
                        switch (targetName){
                            case "Stone Target":
                                visibleTarget = 0;
                                break;
                            case "Blue Rear Bridge":
                                visibleTarget = 1;
                                break;
                            case "Red Rear Bridge":
                                visibleTarget = 2;
                                break;
                            case "Red Front Bridge":
                                visibleTarget = 3;
                                break;
                            case "Blue Front Bridge":
                                visibleTarget = 4;
                                break;
                            case "Red Perimeter 1":
                                visibleTarget = 5;
                                break;
                            case "Red Perimeter 2":
                                visibleTarget = 6;
                                break;
                            case "Front Perimeter 1":
                                visibleTarget = 7;
                                break;
                            case "Front Perimeter 2":
                                visibleTarget = 8;
                                break;
                            case "Blue Perimeter 1":
                                visibleTarget = 9;
                                break;
                            case "Blue Perimeter 2":
                                visibleTarget = 10;
                                break;
                            case "Rear Perimeter 1":
                                visibleTarget = 11;
                                break;
                            case "Rear Perimeter 2":
                                visibleTarget = 12;
                                break;
                        }
                        telemetry.addData("Target ID = ", visibleTarget);

                    }

                    // Provide feedback as to where the robot is located (if we know).
                    if (targetVisible) {
                        // express position (translation) of robot in inches.
                        VectorF translation = lastLocation.getTranslation();
                        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                                translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                        // express the rotation of the robot in degrees.
                        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                        roll = rotation.firstAngle;
                        telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", roll, rotation.secondAngle, rotation.thirdAngle);
                        if (visibleTarget == 7) {
                            /**
                             * Align the robot to be facing the front perimeter
                             */
                            if (roll < 1 && roll > -1){
                                telemetry.addData("Robot roll is ", "perfect");
                            } else if (roll >= 1) {
                                telemetry.addData ("Tell the robot to rotate ", "Left");
                            } else if (roll <= -1){
                                telemetry.addData( "Tell the robot to rotate ", "Right");
                            }
                        }
                    }
                    else {
                        telemetry.addData("Visible Target", "none");
                        telemetry.update();
                    }
                    telemetry.addData("visible Target = ", visibleTarget);
                    telemetry.addData("X Coords = ", robotX);
                    telemetry.addData("Y Coords = ", robotY);
                    telemetry.addData("Robot Bearing = ", robotBearing);
                    telemetry.addData("Robot Roll = ", robotRoll);
                    telemetry.update();
//                        state = State.HALT;
                    break;

                case THIRD_STATE:
                    /**
                     * Provide a description of what this state does
                     * Code goes here
                     */

                    state = State.FOURTH_STATE;
                    break;

                case FOURTH_STATE:
                    /**
                     * Provide a description of what this state does
                     * Code goes here
                     */
                    if (runtime.time() > 24){
                        state = State.END_STATE;
                        break;
                    }

                    for (VuforiaTrackable trackable : allTrackables) {
                        if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                            targetName = trackable.getName();
                            telemetry.addData("Visible Target", targetName);
                            targetVisible = true;

                            // getUpdatedRobotLocation() will return null if no new information is available since
                            // the last time that call was made, or if the trackable is not currently visible.
                            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                            if (robotLocationTransform != null) {
                                lastLocation = robotLocationTransform;
                            }
                        } else {
                            targetVisible = false;
                        }
                    }

                    // Provide feedback as to where the robot is located (if we know).
                    if (targetName == "Front Perimeter 1") {
                        // express position (translation) of robot in inches.
                        VectorF translation = lastLocation.getTranslation();
                        robotX = translation.get(0) / mmPerInch;
                        robotY = translation.get(1) / mmPerInch;
                        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                                translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                        // express the rotation of the robot in degrees.
                        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                        roll = rotation.firstAngle;
                        telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", roll, rotation.secondAngle, rotation.thirdAngle);
                        telemetry.addData("Roll = ", roll);
                        if (stage == 0) {
                            telemetry.addData("currently working on ", "roll");
                            if (roll < 1 && roll > -1) {
                                telemetry.addData("Robot roll is ", "perfect");
                                stage = 1;
                                drive.motorsHalt();
                            } else if (roll >= 1) {
                                telemetry.addData("Tell the robot to rotate ", "Left");
                                robot.motorRF.setPower(-0.1);
                                robot.motorRR.setPower(-0.1);
                            } else if (roll <= -1) {
                                telemetry.addData("Tell the robot to rotate ", "Right");
                                robot.motorLF.setPower(-0.1);
                                robot.motorLR.setPower(-0.1);
                            }
                        }
                        if (stage == 1) {
                            telemetry.addData("currently working on ", "RobotY");
                            if (robotY > -38) {
                                telemetry.addData("Robot strafe to the ", "left");
                                robot.motorLF.setPower(0.1);
                                robot.motorRF.setPower(-0.1);
                                robot.motorLR.setPower(0.1);
                                robot.motorRR.setPower(-0.1);
                            } else if (robotY < -40) {
                                telemetry.addData("Robot strafe to the ", "right");
                                robot.motorLF.setPower(-0.1);
                                robot.motorRF.setPower(0.1);
                                robot.motorLR.setPower(-0.1);
                                robot.motorRR.setPower(0.1);
                            } else {
                                telemetry.addData("Robot is in position = ", "TRUE");
                                stage = 2;
                            }
                        }
                        if (stage == 2) {
                            telemetry.addData("currently working on ", "RobotX");
                            if (robotX > -10) {
                                telemetry.addData("Move the robot ", "forward");
                                robot.motorLF.setPower(-0.1);
                                robot.motorRF.setPower(-0.1);
                                robot.motorLR.setPower(-0.1);
                                robot.motorRR.setPower(-0.1);
                            } else if (robotX < 30) {
                                telemetry.addData("Move the robot ", "back");
                                robot.motorLF.setPower(0.1);
                                robot.motorRF.setPower(0.1);
                                robot.motorLR.setPower(0.1);
                                robot.motorRR.setPower(0.1);
                            } else {
                                telemetry.addData("Robot is parked in the correct place = ", "TRUE");
                                state = State.HALT;
                            }
                        }
                        telemetry.update();
                    }
                    else {
                        telemetry.addData("Visible Target", "none");
                        telemetry.update();
                    }

//                    state = State.FIFTH_STATE;
                    break;

                case FIFTH_STATE:
                    /**
                     * Provide a description of what this state does
                     * Code goes here
                     */
                    if (runtime.time() > 24){
                        state = State.END_STATE;
                        break;
                    }

                    state = State.END_STATE;
                    break;

                case END_STATE:
                    /**
                     * This state will identify the location of the robot on the field and
                     * move it to the parking zone.
                     */

                    drive.motorsHalt();
                    telemetry.addData("State = ", "END_STATE");
                    telemetry.addData("Action = ", "Go and Park");
                    telemetry.update();
                    state = State.HALT;
                    break;

                case HALT:
//                    drive.motorsHalt();               //Stop the motors

                    //Stop the DataLogger
//                    dlStop();

                    // Disable Tracking when we are done;
                    targetsSkyStone.deactivate();

                    //Exit the OpMode
                    requestOpModeStop();
                    break;
            }
        }
    }

    /**
     * Transmit telemetry.
     */
    private void telemetry() {

        telemetry.addData("State", String.valueOf(state));
        telemetry.addData("robotX", String.valueOf((int) robotX));
        telemetry.addData("robotY", String.valueOf((int) robotY));
        telemetry.addData("robotBearing", String.valueOf((int) robotBearing));
        telemetry.addData("robotRoll", String.valueOf((int) robotRoll));
//        telemetry.addData("Current Z Int", String.valueOf(currentZint));
//        telemetry.addData("Z Correction", String.valueOf(zCorrection));
        telemetry.addData("touchSensor", String.valueOf(robot.touchLiftForward.getValue()));
        telemetry.addData("touchSensor", String.valueOf(robot.touchLiftBack.getValue()));
        telemetry.addData("touchSensor", String.valueOf(robot.touchLiftUp.getValue()));
        telemetry.addData("touchSensor", String.valueOf(robot.touchLiftDown.getValue()));
        telemetry.addData("Current Encoder Position", String.valueOf(robot.motorLF.getCurrentPosition()));
        telemetry.update();
    }


    /**
     * Setup the dataLogger
     * The dataLogger takes a set of fields defined here and sets up the file on the Android device
     * to save them to.  We then log data later throughout the class.
     */
    private void createDl() {

        Dl = new DataLogger("AutoMecanumSimpleTest" + runtime.time());
        Dl.addField("runTime ");
        Dl.addField("Alliance ");
        Dl.addField("State ");
        Dl.addField("Procedure ");
        Dl.addField("courseCorrect ");
        Dl.addField("heading ");
        Dl.addField("robotX ");
        Dl.addField("robotY ");
        Dl.addField("robotRoll ");
        Dl.addField("X ");
        Dl.addField("Y ");
        Dl.addField("robotBearing ");
        Dl.addField("visibleTarge ");
        Dl.addField("initZ ");
        Dl.addField("currentZ ");
        Dl.addField("zCorrection ");
        Dl.addField("touchSensor ");
        Dl.addField("ODS ");
        Dl.addField("colorRightRed ");
        Dl.addField("colorRightBlue ");
        Dl.addField("colorLeftRed ");
        Dl.addField("colorLeftBlue ");
        Dl.addField("LFTargetPos ");
        Dl.addField("LFMotorPos ");
        Dl.addField("motorLF ");
        Dl.addField("motorRF ");
        Dl.addField("mtoroLR ");
        Dl.addField("motorRR ");
        Dl.newLine();
    }

    /**
     * Log data to the file on the phone.
     */
    private void logData() {

        Dl.addField(String.valueOf(runtime.time()));
        Dl.addField(String.valueOf(state));
        Dl.addField(String.valueOf((int) robotX));
        Dl.addField(String.valueOf((int) robotY));
        Dl.addField(String.valueOf((int) robotRoll));
        Dl.addField(String.valueOf((int) robotBearing));
        Dl.addField(String.valueOf((int) visibleTarget));
//        Dl.addField(String.valueOf(currentZint));
        Dl.addField(String.valueOf(robot.rangeSensorFront));
        Dl.addField(String.valueOf(robot.touchLiftDown.getValue()));
        Dl.addField(String.valueOf(robot.touchLiftUp.getValue()));
        Dl.addField(String.valueOf(robot.touchLiftForward.getValue()));
        Dl.addField(String.valueOf(robot.touchLiftBack.getValue()));
        Dl.addField(String.valueOf(robot.motorLF.getCurrentPosition()));
        Dl.newLine();
    }

    /**
     * Stop the DataLogger
     */
    private void dlStop() {
        Dl.closeDataLogger();

    }

    enum State {
        FOUNDATION, SECOND_STATE, THIRD_STATE, FOURTH_STATE,
        FIFTH_STATE, PARK, HALT, END_STATE, VUFORIA_TEST
    }
}
