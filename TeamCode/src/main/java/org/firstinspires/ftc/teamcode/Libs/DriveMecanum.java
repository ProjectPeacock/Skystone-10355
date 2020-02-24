package org.firstinspires.ftc.teamcode.Libs;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import java.util.List;


/*
 * FTC Team 10355 driveMecanum Class for 2019-2020 Skystone season.
 */

public class DriveMecanum {
    private double myCurrentMotorPosition;
    private double LF, RF, LR, RR;
    private List<VuforiaTrackable> myTrackables;
    private HardwareProfile robot;
    private LinearOpMode opMode;
    private ElapsedTime runtime = new ElapsedTime();
    private skystoneVuforia myVuforia;
    private List<Double> vuforiaTracking;
    private double robotX;          // The robot's X position from VuforiaLib
    private double robotY;  // The robot's Y position from VuforiaLib
    private double robotBearing;    //Bearing to, i.e. the bearing you need to steer toward


    public DriveMecanum(HardwareProfile myRobot, LinearOpMode myOpMode, skystoneVuforia thisVuforia,
                        List<VuforiaTrackable> trackableList) {
        robot = myRobot;
        opMode = myOpMode;
        myVuforia = thisVuforia;
        myTrackables = trackableList;
    }

    public void translateVuforiaNavY(double power, double heading, double timeOut, double y) {
        double changeSpeed = 0;
        double initZ = robot.mrGyro.getIntegratedZValue();
        double currentZint;

        heading = heading * (Math.PI / 180);

        vuforiaTracking = myVuforia.getLocation(myTrackables);
        robotX = vuforiaTracking.get(0);
        robotY = vuforiaTracking.get(1);
        robotBearing = vuforiaTracking.get(2);

        while (opMode.opModeIsActive() && runtime.time() < timeOut && robotY < y) {
            opMode.telemetry.addData("if (y > startingY)", "");
            LF = power * Math.sin(heading + (Math.PI / 4)) + changeSpeed;
            RF = power * Math.cos(heading + (Math.PI / 4)) - changeSpeed;
            LR = power * Math.cos(heading + (Math.PI / 4)) + changeSpeed;
            RR = power * Math.sin(heading + (Math.PI / 4)) - changeSpeed;

            if (LF > 1 || LF < -1) {
                LF = 0;
            }

            if (RF > 1 || RF < -1) {
                RF = 0;
            }

            if (LR > 1 || LR < -1) {
                LR = 0;
            }

            if (RR > 1 || RR < -1) {
                RR = 0;
            }

            currentZint = robot.mrGyro.getIntegratedZValue();

            if (currentZint != initZ) {  //Robot has drifted off course
                double zCorrection = Math.abs(initZ - currentZint);

                if (heading > 180 && heading < 359.99999) {
                    if (currentZint > initZ) {  //Robot has drifted left
                        LF = LF + (zCorrection / 100);
                        RF = RF - (zCorrection / 100);
                        LR = LR + (zCorrection / 100);
                        RR = RR - (zCorrection / 100);

                    }

                    if (currentZint < initZ) {  //Robot has drifted right
                        LF = LF - (zCorrection / 100);
                        RF = RF + (zCorrection / 100);
                        LR = LR - (zCorrection / 100);
                        RR = RR + (zCorrection / 100);
                    }
                }

                if (heading > 0 && heading < 180) {
                    if (currentZint > initZ) {  //Robot has drifted left
                        LF = LF + (zCorrection / 100);
                        RF = RF - (zCorrection / 100);
                        LR = LR + (zCorrection / 100);
                        RR = RR - (zCorrection / 100);
                    }

                    if (currentZint < initZ) {  //Robot has drifted right
                        LF = LF - (zCorrection / 100);
                        RF = RF + (zCorrection / 100);
                        LR = LR - (zCorrection / 100);
                        RR = RR + (zCorrection / 100);
                    }
                }

                if (heading == 0) {
                    if (currentZint > initZ) {  //Robot has drifted left
                        LF = LF + (zCorrection / 100);
                        RF = RF - (zCorrection / 100);
                        LR = LR + (zCorrection / 100);
                        RR = RR - (zCorrection / 100);
                    }

                    if (currentZint < initZ) {  //Robot has drifted right
                        LF = LF - (zCorrection / 100);
                        RF = RF + (zCorrection / 100);
                        LR = LR - (zCorrection / 100);
                        RR = RR + (zCorrection / 100);
                    }
                }

                if (heading == 180) {
                    if (currentZint > initZ) {  //Robot has drifted left
                        LF = LF - (zCorrection / 100);
                        RF = RF + (zCorrection / 100);
                        LR = LR - (zCorrection / 100);
                        RR = RR + (zCorrection / 100);
                    }

                    if (currentZint < initZ) {  //Robot has drifted right
                        LF = LF + (zCorrection / 100);
                        RF = RF - (zCorrection / 100);
                        LR = LR + (zCorrection / 100);
                        RR = RR - (zCorrection / 100);
                    }
                }
            }

            robot.motorLF.setPower(LF);
            robot.motorRF.setPower(RF);
            robot.motorLR.setPower(LR);
            robot.motorRR.setPower(RR);

            myCurrentMotorPosition = robot.motorLR.getCurrentPosition();

            vuforiaTracking = myVuforia.getLocation(myTrackables);
            robotX = vuforiaTracking.get(0);
            robotY = vuforiaTracking.get(1);
            robotBearing = vuforiaTracking.get(2);

            opMode.telemetry.addData("Status", "Run Time: " + runtime.time());
            opMode.telemetry.addData("Target Y", String.valueOf(y));
            opMode.telemetry.addData("robotX", String.valueOf(robotX));
            opMode.telemetry.addData("robotY", String.valueOf(robotY));
            opMode.telemetry.addData("robotBearing", String.valueOf(robotBearing));
            opMode.telemetry.addData("LF", String.valueOf(LF));
            opMode.telemetry.addData("RF", String.valueOf(RF));
            opMode.telemetry.addData("LR", String.valueOf(LR));
            opMode.telemetry.addData("RR", String.valueOf(RR));
            opMode.telemetry.update();

            opMode.idle();
        }

        motorsHalt();

    }

    public void translateVuforiaNavX(double power, double heading, double timeOut, double x) {
        double initZ = robot.mrGyro.getIntegratedZValue();
        double currentZint;
        double startingX;

        heading = heading * (Math.PI / 180);

        vuforiaTracking = myVuforia.getLocation(myTrackables);
        robotX = vuforiaTracking.get(0);
        robotY = vuforiaTracking.get(1);
        robotBearing = vuforiaTracking.get(2);

        startingX = robotX;

        if (x < startingX) {
            while (opMode.opModeIsActive() && runtime.time() < timeOut && robotX > x) {

                LF = power * Math.sin(heading + (Math.PI / 4));
                RF = power * Math.cos(heading + (Math.PI / 4));
                LR = power * Math.cos(heading + (Math.PI / 4));
                RR = power * Math.sin(heading + (Math.PI / 4));

                if (LF > 1 || LF < -1) {
                    LF = 0;
                }

                if (RF > 1 || RF < -1) {
                    RF = 0;
                }

                if (LR > 1 || LR < -1) {
                    LR = 0;
                }

                if (RR > 1 || RR < -1) {
                    RR = 0;
                }

                currentZint = robot.mrGyro.getIntegratedZValue();

                if (currentZint != initZ) {  //Robot has drifted off course
                    double zCorrection = Math.abs(initZ - currentZint);

                    if (heading > 180 && heading < 359.99999) {
                        if (currentZint > initZ) {  //Robot has drifted left
                            LF = LF + (zCorrection / 100);
                            RF = RF - (zCorrection / 100);
                            LR = LR + (zCorrection / 100);
                            RR = RR - (zCorrection / 100);
                        }

                        if (currentZint < initZ) {  //Robot has drifted right
                            LF = LF - (zCorrection / 100);
                            RF = RF + (zCorrection / 100);
                            LR = LR - (zCorrection / 100);
                            RR = RR + (zCorrection / 100);
                        }
                    }

                    if (heading > 0 && heading < 180) {
                        if (currentZint > initZ) {  //Robot has drifted left
                            LF = LF + (zCorrection / 100);
                            RF = RF + (zCorrection / 100);
                            LR = LR - (zCorrection / 100);
                            RR = RR - (zCorrection / 100);
                        }

                        if (currentZint < initZ) {  //Robot has drifted right
                            LF = LF - (zCorrection / 100);
                            RF = RF - (zCorrection / 100);
                            LR = LR + (zCorrection / 100);
                            RR = RR + (zCorrection / 100);
                        }
                    }

                    if (heading == 0) {
                        if (currentZint > initZ) {  //Robot has drifted left
                            LF = LF + (zCorrection / 100);
                            RF = RF - (zCorrection / 100);
                            LR = LR + (zCorrection / 100);
                            RR = RR - (zCorrection / 100);
                        }

                        if (currentZint < initZ) {  //Robot has drifted right
                            LF = LF - (zCorrection / 100);
                            RF = RF + (zCorrection / 100);
                            LR = LR - (zCorrection / 100);
                            RR = RR + (zCorrection / 100);
                        }
                    }

                    if (heading == 180) {
                        if (currentZint > initZ) {  //Robot has drifted left
                            LF = LF - (zCorrection / 100);
                            RF = RF + (zCorrection / 100);
                            LR = LR - (zCorrection / 100);
                            RR = RR + (zCorrection / 100);
                        }

                        if (currentZint < initZ) {  //Robot has drifted right
                            LF = LF + (zCorrection / 100);
                            RF = RF - (zCorrection / 100);
                            LR = LR + (zCorrection / 100);
                            RR = RR - (zCorrection / 100);
                        }
                    }
                }
            }
            robot.motorLF.setPower(LF);
            robot.motorRF.setPower(RF);
            robot.motorLR.setPower(LR);
            robot.motorRR.setPower(RR);

            myCurrentMotorPosition = robot.motorLR.getCurrentPosition();

            vuforiaTracking = myVuforia.getLocation(myTrackables);
            robotX = vuforiaTracking.get(0);
            robotY = vuforiaTracking.get(1);
            robotBearing = vuforiaTracking.get(2);

            opMode.telemetry.addData("Status", "Run Time: " + runtime.time());
            opMode.telemetry.addData("robotX", String.valueOf(robotX));
            opMode.telemetry.addData("robotY", String.valueOf(robotY));
            opMode.telemetry.addData("robotBearing", String.valueOf(robotBearing));
            opMode.telemetry.addData("LF", String.valueOf(LF));
            opMode.telemetry.addData("RF", String.valueOf(RF));
            opMode.telemetry.addData("LR", String.valueOf(LR));
            opMode.telemetry.addData("RR", String.valueOf(RR));
            opMode.telemetry.update();

            opMode.idle();
        }

        motorsHalt();
    }

    public void translateTime(double power, double heading, double timeOut) {
        double initZ = robot.mrGyro.getIntegratedZValue();
        double currentZint;
        double timeElapsed;
        double runtimeValue;

        runtimeValue = runtime.time();
        timeElapsed = runtimeValue - runtime.time();
        heading = heading * (Math.PI / 180);

        while (opMode.opModeIsActive() && timeElapsed < timeOut) {

            LF = power * Math.sin(heading + (Math.PI / 4));
            RF = power * Math.cos(heading + (Math.PI / 4));
            LR = power * Math.cos(heading + (Math.PI / 4));
            RR = power * Math.sin(heading + (Math.PI / 4));

            if (LF > 1 || LF < -1) {
                LF = 0;
            }

            if (RF > 1 || RF < -1) {
                RF = 0;
            }

            if (LR > 1 || LR < -1) {
                LR = 0;
            }

            if (RR > 1 || RR < -1) {
                RR = 0;
            }

            currentZint = robot.mrGyro.getIntegratedZValue();

            if (currentZint != initZ) {  //Robot has drifted off course
                double zCorrection = Math.abs(initZ - currentZint);

                if (heading > 180 && heading < 359.99999) {
                    if (currentZint > initZ) {  //Robot has drifted left
                        opMode.telemetry.addData("Robot has drifted ", "LEFT");
                        LF = LF + (zCorrection / 100);
                        RF = RF - (zCorrection / 100);
                        LR = LR + (zCorrection / 100);
                        RR = RR - (zCorrection / 100);
                    }

                    if (currentZint < initZ) {  //Robot has drifted right
                        opMode.telemetry.addData("Robot has drifted ", "RIGHT");
                        LF = LF - (zCorrection / 100);
                        RF = RF + (zCorrection / 100);
                        LR = LR - (zCorrection / 100);
                        RR = RR + (zCorrection / 100);
                    }
                }

                if (heading > 0 && heading < 180) {
                    if (currentZint > initZ) {  //Robot has drifted left
                        opMode.telemetry.addData("Robot has drifted ", "LEFT");
                        LF = LF + (zCorrection / 100);
                        RF = RF - (zCorrection / 100);
                        LR = LR + (zCorrection / 100);
                        RR = RR - (zCorrection / 100);
                    }

                    if (currentZint < initZ) {  //Robot has drifted right
                        opMode.telemetry.addData("Robot has drifted ", "RIGHT");
                        LF = LF - (zCorrection / 100);
                        RF = RF + (zCorrection / 100);
                        LR = LR - (zCorrection / 100);
                        RR = RR + (zCorrection / 100);
                    }
                }

                if (heading == 0) {
                    if (currentZint > initZ) {  //Robot has drifted left
                        opMode.telemetry.addData("Robot has drifted ", "LEFT");
                        LF = LF + (zCorrection / 100);
                        RF = RF - (zCorrection / 100);
                        LR = LR + (zCorrection / 100);
                        RR = RR - (zCorrection / 100);
                    }

                    if (currentZint < initZ) {  //Robot has drifted right
                        opMode.telemetry.addData("Robot has drifted ", "RIGHT");
                        LF = LF - (zCorrection / 100);
                        RF = RF + (zCorrection / 100);
                        LR = LR - (zCorrection / 100);
                        RR = RR + (zCorrection / 100);
                    }
                }

                if (heading == 180) {
                    if (currentZint > initZ) {  //Robot has drifted left
                        opMode.telemetry.addData("Robot has drifted ", "LEFT");
                        LF = LF - (zCorrection / 100);
                        RF = RF + (zCorrection / 100);
                        LR = LR - (zCorrection / 100);
                        RR = RR + (zCorrection / 100);
                    }

                    if (currentZint < initZ) {  //Robot has drifted right
                        opMode.telemetry.addData("Robot has drifted ", "RIGHT");
                        LF = LF + (zCorrection / 100);
                        RF = RF - (zCorrection / 100);
                        LR = LR + (zCorrection / 100);
                        RR = RR - (zCorrection / 100);
                    }
                }
            }
            robot.motorLF.setPower(LF);
            robot.motorRF.setPower(RF);
            robot.motorLR.setPower(LR);
            robot.motorRR.setPower(RR);

            myCurrentMotorPosition = robot.motorLR.getCurrentPosition();

            timeElapsed = runtime.time() - runtimeValue;
            opMode.telemetry.addData("Status", "Run Time: " + runtime.time());
            opMode.telemetry.addData("Status", "Elapsed Time: " + timeElapsed);
            opMode.telemetry.addData("Gyro Value", String.valueOf(currentZint));
            opMode.telemetry.addData("intZ", String.valueOf(initZ));

            opMode.telemetry.addData("LF", String.valueOf(LF));
            opMode.telemetry.addData("RF", String.valueOf(RF));
            opMode.telemetry.addData("LR", String.valueOf(LR));
            opMode.telemetry.addData("RR", String.valueOf(RR));
            opMode.telemetry.update();

            opMode.idle();
        }
        motorsHalt();
    }

    public void swivelGrab(){

        robot.servoSwivel.setPower(-0.4);
        robot.servoGrab.setPower(0.90);
        robot.servo4Bar1.setPower(0.8);       // raise arm to reset swivel and grabber
        robot.servo4Bar2.setPower(-0.8);
        if (!robot.touchLiftForward.isPressed()){
            robot.motorLinear.setPower(0.4);}
        else {
            robot.motorLinear.setPower(0);}
        opMode.sleep(200);
        robot.servo4Bar1.setPower(-0.1);       // lower the arm to grab the stone
        robot.servo4Bar2.setPower(0.1);
        opMode.sleep(300);
        robot.servoGrab.setPower(0.90);
    }

    public void fullDrive(){
        robot.motorLF.setPower(-0.5);
        robot.motorRF.setPower(-0.5);
        robot.motorRR.setPower(-0.5);
        robot.motorLR.setPower(-0.5);
        opMode.sleep(300);
    }

    public void rightStrafe(){
        robot.motorLF.setPower(0.5);
        robot.motorRF.setPower(-0.5);
        robot.motorRR.setPower(-0.5);
        robot.motorLR.setPower(0.5);
        opMode.sleep(200);
    }

    public void swivelPlace(){
        robot.servoGrab.setPower(0.5);
    }

    public void swivelReady(){

        robot.motorLift.setPower(-1);  // Rotate the stone into position to place
        robot.servoGrab.setPower(0.90);
        opMode.sleep(300);
        robot.servo4Bar1.setPower(0.8);       // lower the arm to grab the stone
        robot.servo4Bar2.setPower(-0.8);
        opMode.sleep(100);
        robot.motorLift.setPower(0);
        robot.servoSwivel.setPower(1);
    }


    public void translateSkystone(double power, double heading, double maxTime) {
        double initZ = robot.mrGyro.getIntegratedZValue();
        double currentZint;
        double timeElapsed;
        double runtimeValue;
        int stoneColor;
        boolean color = false;
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
     //   int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
     //   final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        /*
         * In the case where the sensor fails or the sensor is too far away from the stones to
         * detect the skystone, we want the function to abort the effrt and go to grab the closest
         * stone and finish the autonomous mode.  To do this, we track the time the algorithm runs
         * and tell it to cancel if the maxTime is exceeded.
         */
        runtimeValue = runtime.time();
        timeElapsed = runtime.time() - runtimeValue;
        heading = heading * (Math.PI / 180);

        while (opMode.opModeIsActive() && (!color) && (timeElapsed < maxTime)) {

            LF = power * Math.sin(heading + (Math.PI / 4));
            RF = power * Math.cos(heading + (Math.PI / 4));
            LR = power * Math.cos(heading + (Math.PI / 4));
            RR = power * Math.sin(heading + (Math.PI / 4));

            Color.RGBToHSV((int) (robot.colorSensorRevStone.red() * SCALE_FACTOR),
                    (int) (robot.colorSensorRevStone.green() * SCALE_FACTOR),
                    (int) (robot.colorSensorRevStone.blue() * SCALE_FACTOR),
                    hsvValues);

            stoneColor = Color.HSVToColor(0xff, values);
            if (stoneColor <= -20000) {
                opMode.telemetry.addData("Color Sensor Sees : ", "Yellow");
            }
            else if(stoneColor >= -10000) {
                opMode.telemetry.addData("Color Sensor Sees : ", "Black");
                color = true;
            }
            else {
                opMode.telemetry.addData("Color Sensor Sees : ", "Space");
            }

            if (LF > 1 || LF < -1) {
                LF = 0;
            }

            if (RF > 1 || RF < -1) {
                RF = 0;
            }

            if (LR > 1 || LR < -1) {
                LR = 0;
            }

            if (RR > 1 || RR < -1) {
                RR = 0;
            }

            currentZint = robot.mrGyro.getIntegratedZValue();

            if (currentZint != initZ) {  //Robot has drifted off course
                double zCorrection = Math.abs(initZ - currentZint);

                if (heading > 180 && heading < 359.99999) {
                    if (currentZint > initZ) {  //Robot has drifted left
                        LF = LF + (zCorrection / 100);
                        RF = RF - (zCorrection / 100);
                        LR = LR + (zCorrection / 100);
                        RR = RR - (zCorrection / 100);
                    }

                    if (currentZint < initZ) {  //Robot has drifted right
                        LF = LF - (zCorrection / 100);
                        RF = RF + (zCorrection / 100);
                        LR = LR - (zCorrection / 100);
                        RR = RR + (zCorrection / 100);
                    }
                }

                if (heading > 0 && heading < 180) {
                    if (currentZint > initZ) {  //Robot has drifted left
                        LF = LF + (zCorrection / 100);
                        RF = RF - (zCorrection / 100);
                        LR = LR + (zCorrection / 100);
                        RR = RR - (zCorrection / 100);
                    }

                    if (currentZint < initZ) {  //Robot has drifted right
                        LF = LF - (zCorrection / 100);
                        RF = RF + (zCorrection / 100);
                        LR = LR - (zCorrection / 100);
                        RR = RR + (zCorrection / 100);
                    }
                }

                if (heading == 0) {
                    if (currentZint > initZ) {  //Robot has drifted left
                        LF = LF + (zCorrection / 100);
                        RF = RF - (zCorrection / 100);
                        LR = LR + (zCorrection / 100);
                        RR = RR - (zCorrection / 100);
                    }

                    if (currentZint < initZ) {  //Robot has drifted right
                        LF = LF - (zCorrection / 100);
                        RF = RF + (zCorrection / 100);
                        LR = LR - (zCorrection / 100);
                        RR = RR + (zCorrection / 100);
                    }
                }

                if (heading == 180) {
                    if (currentZint > initZ) {  //Robot has drifted left
                        LF = LF - (zCorrection / 100);
                        RF = RF + (zCorrection / 100);
                        LR = LR - (zCorrection / 100);
                        RR = RR + (zCorrection / 100);
                    }

                    if (currentZint < initZ) {  //Robot has drifted right
                        LF = LF + (zCorrection / 100);
                        RF = RF - (zCorrection / 100);
                        LR = LR + (zCorrection / 100);
                        RR = RR - (zCorrection / 100);
                    }
                }
            }
            robot.motorLF.setPower(LF);
            robot.motorRF.setPower(RF);
            robot.motorLR.setPower(LR);
            robot.motorRR.setPower(RR);

            myCurrentMotorPosition = robot.motorLR.getCurrentPosition();

            timeElapsed = runtime.time() - runtimeValue;
            opMode.telemetry.addData("Status", "Run Time: " + runtime.time());
            opMode.telemetry.addData("Status", "Elapsed Time: " + timeElapsed);
            opMode.telemetry.addData("Color sensor Value", String.valueOf(robot.colorSensorRevStone.red()));
            opMode.telemetry.addData("Gyro Value", String.valueOf(currentZint));
            opMode.telemetry.addData("intZ", String.valueOf(initZ));
            opMode.telemetry.addData("LF", String.valueOf(LF));
            opMode.telemetry.addData("RF", String.valueOf(RF));
            opMode.telemetry.addData("LR", String.valueOf(LR));
            opMode.telemetry.addData("RR", String.valueOf(RR));
            opMode.telemetry.update();

            opMode.idle();
        }
        motorsHalt();

    }
    /**
     * robotCorrect will use the range sensor to measure distance from the wall and will drive
     * to the specified distance.
     * @param power     // controls speed at which the robot should move
     * @param heading   // direction to head (will strafe to the location
     * @param distance  // distance to the wall that the robot is trying to acheive
     * @param maxTime   // maximum time that the function should run - exits at maxTime
     */
    public void robotCorrect(double power, double heading, double distance, double maxTime){
        double initZ = robot.mrGyro.getIntegratedZValue();
        double currentZint;
        boolean active = true;
        double timeElapsed;
        double runtimeValue;
        double currentRangeB;
        double currentRangeF;
        double currentRangeL;
        double currentRangeR;

        runtimeValue = runtime.time();
        timeElapsed = runtime.time() - runtimeValue;
        heading = heading * (Math.PI / 180);

        currentRangeR = robot.rightRange.getDistance(DistanceUnit.CM);
        currentRangeB = robot.rearRange.getDistance(DistanceUnit.CM);
        currentRangeF = robot.wallRangeSensor.getDistance(DistanceUnit.CM);
        currentRangeL = robot.leftRange.getDistance(DistanceUnit.CM);

        if (currentRangeB > 60) {
            active = false;
        }

        if (currentRangeB < 60 && currentRangeF < 18) {
            active = true;
        }
        while (opMode.opModeIsActive() && active ){
            if (currentRangeB < 60 && currentRangeF < 18){
                translateTime(0.6, 90, 1);

                while (currentRangeR < 5){
                    opMode.sleep(250);
                }
                translateTime(0.6, 270, 0.6);
                active = false;
            }
        }
    }

    /**
     * translateFromWall will use the range sensor to measure distance from the wall and will drive
     * to the specified distance.
     * @param power     // controls speed at which the robot should move
     * @param heading   // direction to head (will strafe to the location
     * @param distance  // distance to the wall that the robot is trying to acheive
     * @param maxTime   // maximum time that the function should run - exits at maxTime
     */

    public void translateFromWall(String sensor, double power, double heading, double distance, double maxTime) {
        double initZ = robot.mrGyro.getIntegratedZValue();
        double currentZint;
        double currentRange;
        boolean active = true;
        double timeElapsed;
        double runtimeValue;

        /*
         * In the case where the sensor fails or the sensor is too far away from the stones to
         * detect the skystone, we want the function to abort the effrt and go to grab the closest
         * stone and finish the autonomous mode.  To do this, we track the time the algorithm runs
         * and tell it to cancel if the maxTime is exceeded.
         */
        runtimeValue = runtime.time();
        timeElapsed = runtime.time() - runtimeValue;
        heading = heading * (Math.PI / 180);

        currentRange = currentRangeValue(sensor);   // get the value from the correct sensor

        if (currentRange > distance) active = false;

        while (opMode.opModeIsActive() && active & (timeElapsed < maxTime)) {

            LF = power * Math.sin(heading + (Math.PI / 4));
            RF = power * Math.cos(heading + (Math.PI / 4));
            LR = power * Math.cos(heading + (Math.PI / 4));
            RR = power * Math.sin(heading + (Math.PI / 4));

            if (LF > 1 || LF < -1) {
                LF = 0;
            }

            if (RF > 1 || RF < -1) {
                RF = 0;
            }

            if (LR > 1 || LR < -1) {
                LR = 0;
            }

            if (RR > 1 || RR < -1) {
                RR = 0;
            }

            currentZint = robot.mrGyro.getIntegratedZValue();

            if (currentZint != initZ) {  //Robot has drifted off course
                double zCorrection = Math.abs(initZ - currentZint);

                if (heading > 180 && heading < 359.99999) {
                    if (currentZint > initZ) {  //Robot has drifted left
                        LF = LF + (zCorrection / 100);
                        RF = RF - (zCorrection / 100);
                        LR = LR + (zCorrection / 100);
                        RR = RR - (zCorrection / 100);
                    }

                    if (currentZint < initZ) {  //Robot has drifted right
                        LF = LF - (zCorrection / 100);
                        RF = RF + (zCorrection / 100);
                        LR = LR - (zCorrection / 100);
                        RR = RR + (zCorrection / 100);
                    }
                }

                if (heading > 0 && heading < 180) {
                    if (currentZint > initZ) {  //Robot has drifted left
                        LF = LF + (zCorrection / 100);
                        RF = RF - (zCorrection / 100);
                        LR = LR + (zCorrection / 100);
                        RR = RR - (zCorrection / 100);
                    }

                    if (currentZint < initZ) {  //Robot has drifted right
                        LF = LF - (zCorrection / 100);
                        RF = RF + (zCorrection / 100);
                        LR = LR - (zCorrection / 100);
                        RR = RR + (zCorrection / 100);
                    }
                }

                if (heading == 0) {
                    if (currentZint > initZ) {  //Robot has drifted left
                        LF = LF + (zCorrection / 100);
                        RF = RF - (zCorrection / 100);
                        LR = LR + (zCorrection / 100);
                        RR = RR - (zCorrection / 100);
                    }

                    if (currentZint < initZ) {  //Robot has drifted right
                        LF = LF - (zCorrection / 100);
                        RF = RF + (zCorrection / 100);
                        LR = LR - (zCorrection / 100);
                        RR = RR + (zCorrection / 100);
                    }
                }

                if (heading == 180) {
                    if (currentZint > initZ) {  //Robot has drifted left
                        LF = LF - (zCorrection / 100);
                        RF = RF + (zCorrection / 100);
                        LR = LR - (zCorrection / 100);
                        RR = RR + (zCorrection / 100);
                    }

                    if (currentZint < initZ) {  //Robot has drifted right
                        LF = LF + (zCorrection / 100);
                        RF = RF - (zCorrection / 100);
                        LR = LR + (zCorrection / 100);
                        RR = RR - (zCorrection / 100);
                    }
                }
            }
            robot.motorLF.setPower(LF);
            robot.motorRF.setPower(RF);
            robot.motorLR.setPower(LR);
            robot.motorRR.setPower(RR);

            myCurrentMotorPosition = robot.motorLR.getCurrentPosition();

            // check to see if the distance traveled is less than the range specification
            currentRange = currentRangeValue(sensor);   // get the value from the correct sensor
            if (currentRange > distance) active = false;

            opMode.telemetry.addData("Proceedure = ", "TranslateFromWall");
            opMode.telemetry.addData("Range Value = ", currentRange);
            opMode.telemetry.addData("Status", "Run Time: " + runtime.time());
            opMode.telemetry.addData("LF", String.valueOf(LF));
            opMode.telemetry.addData("RF", String.valueOf(RF));
            opMode.telemetry.addData("LR", String.valueOf(LR));
            opMode.telemetry.addData("RR", String.valueOf(RR));
            opMode.telemetry.update();

            opMode.idle();
        }
        motorsHalt();
    }

    public double currentRangeValue(String sensor){
        double range;

        if (sensor.equals("front")){
            range = robot.wallRangeSensor.getDistance(DistanceUnit.CM);
        } else if (sensor.equals("left")){
            range = robot.leftRange.getDistance(DistanceUnit.CM);
        } else if (sensor.equals("right")){
            range = robot.rightRange.getDistance(DistanceUnit.CM);
        } else if (sensor.equals("rear")){
            range = robot.rearRange.getDistance(DistanceUnit.CM);
        } else {
            range = 0;
        }
        return range;
    }
    /**
     * translateToWall will use the range sensor to measure distance to the wall and will drive
     * to the specified distance.
     * @param power     // controls speed at which the robot should move
     * @param heading   // direction to head (will strafe to the location
     * @param distance  // distance to the wall that the robot is trying to acheive
     * @param maxTime   // maximum time that the function should run - exits at maxTime
     */
    public void translateToWall(double power, double heading, double distance, String sensor, double maxTime) {
        double initZ = robot.mrGyro.getIntegratedZValue();
        double currentZint;
        double currentRange;
        boolean active = true;
        double timeElapsed;
        double runtimeValue;

        /*
         * In the case where the sensor fails or the sensor is too far away from the stones to
         * detect the skystone, we want the function to abort the effrt and go to grab the closest
         * stone and finish the autonomous mode.  To do this, we track the time the algorithm runs
         * and tell it to cancel if the maxTime is exceeded.
         */
        runtimeValue = runtime.time();
        timeElapsed = runtime.time() - runtimeValue;
        heading = heading * (Math.PI / 180);

        currentRange = currentRangeValue(sensor);   // get the value from the correct sensor
        if (currentRange < distance) active = false;

        while (opMode.opModeIsActive() && active & (timeElapsed < maxTime)) {

            LF = power * Math.sin(heading + (Math.PI / 4));
            RF = power * Math.cos(heading + (Math.PI / 4));
            LR = power * Math.cos(heading + (Math.PI / 4));
            RR = power * Math.sin(heading + (Math.PI / 4));

            if (LF > 1 || LF < -1) {
                LF = 0;
            }

            if (RF > 1 || RF < -1) {
                RF = 0;
            }

            if (LR > 1 || LR < -1) {
                LR = 0;
            }

            if (RR > 1 || RR < -1) {
                RR = 0;
            }

            currentZint = robot.mrGyro.getIntegratedZValue();

            if (currentZint != initZ) {  //Robot has drifted off course
                double zCorrection = Math.abs(initZ - currentZint);
                    opMode.telemetry.addData("currentInt", currentZint);
                    opMode.telemetry.addData("Zint", initZ);
                if (heading > 180 && heading < 359.99999) {
                    if (currentZint > initZ) {  //Robot has drifted left
                        LF = LF + (zCorrection / 100);
                        RF = RF - (zCorrection / 100);
                        LR = LR + (zCorrection / 100);
                        RR = RR - (zCorrection / 100);
                    }

                    if (currentZint < initZ) {  //Robot has drifted right
                        LF = LF - (zCorrection / 100);
                        RF = RF + (zCorrection / 100);
                        LR = LR - (zCorrection / 100);
                        RR = RR + (zCorrection / 100);
                    }

                }

                if (heading > 0 && heading < 180) {
                    if (currentZint > initZ) {  //Robot has drifted left
                        LF = LF + (zCorrection / 100);
                        RF = RF - (zCorrection / 100);
                        LR = LR + (zCorrection / 100);
                        RR = RR - (zCorrection / 100);
                    }

                    if (currentZint < initZ) {  //Robot has drifted right
                        LF = LF - (zCorrection / 100);
                        RF = RF + (zCorrection / 100);
                        LR = LR - (zCorrection / 100);
                        RR = RR + (zCorrection / 100);
                    }
                }

                if (heading == 0) {
                    if (currentZint > initZ) {  //Robot has drifted left
                        LF = LF + (zCorrection / 100);
                        RF = RF - (zCorrection / 100);
                        LR = LR + (zCorrection / 100);
                        RR = RR - (zCorrection / 100);
                    }

                    if (currentZint < initZ) {  //Robot has drifted right
                        LF = LF - (zCorrection / 100);
                        RF = RF + (zCorrection / 100);
                        LR = LR - (zCorrection / 100);
                        RR = RR + (zCorrection / 100);
                    }
                }

                if (heading == 180) {
                    if (currentZint > initZ) {  //Robot has drifted left
                        LF = LF - (zCorrection / 100);
                        RF = RF + (zCorrection / 100);
                        LR = LR - (zCorrection / 100);
                        RR = RR + (zCorrection / 100);
                    }

                    if (currentZint < initZ) {  //Robot has drifted right
                        LF = LF + (zCorrection / 100);
                        RF = RF - (zCorrection / 100);
                        LR = LR + (zCorrection / 100);
                        RR = RR - (zCorrection / 100);
                    } }
            }
            robot.motorLF.setPower(LF);
            robot.motorRF.setPower(RF);
            robot.motorLR.setPower(LR);
            robot.motorRR.setPower(RR);

            myCurrentMotorPosition = robot.motorLR.getCurrentPosition();

            // check to see if the distance traveled is less than the range specification
            currentRange = currentRangeValue(sensor);   // get the value from the correct sensor
            if (currentRange < distance) active = false;

            timeElapsed = runtime.time() - runtimeValue;
            opMode.telemetry.addData("Proceedure = ", "TranslateToWall");
            opMode.telemetry.addData("Range value = ", currentRange);
            opMode.telemetry.addData("Status", "Run Time: " + runtime.time());
            opMode.telemetry.addData("Time Elapsed", timeElapsed);
            opMode.telemetry.addData("LF", String.valueOf(LF));
            opMode.telemetry.addData("RF", String.valueOf(RF));
            opMode.telemetry.addData("LR", String.valueOf(LR));
            opMode.telemetry.addData("RR", String.valueOf(RR));

            opMode.telemetry.update();

            opMode.idle();
        }
        motorsHalt();
    }

    /*
     * Raise the lift up
     */
    public void raiseLift(double maxTime){
        double timeElapsed;
        double runtimeValue;

        /*
         * In the case where the sensor fails or the sensor is too far away from the stones to
         * detect the skystone, we want the function to abort the effort and go to grab the closest
         * stone and finish the autonomous mode.  To do this, we track the time the algorithm runs
         * and tell it to cancel if the maxTime is exceeded.
         */
        runtimeValue = runtime.time();
        timeElapsed = runtime.time() - runtimeValue;

        // turn the linear motor on to begin raising the lift
        robot.motorLinear.setPower(0.4);


        while (opMode.opModeIsActive() && (!robot.touchLiftForward.isPressed()) && (timeElapsed < maxTime)){
            timeElapsed = runtime.time() - runtimeValue;
            // wait for the lift to lean all the way forward
            opMode.telemetry.addData("Status", "Run Time: " + timeElapsed);
            opMode.telemetry.update();
        }
        robot.motorLinear.setPower(0);  // shut the motor off
    }

    /*
     * Raise the lift up
     */
    public void lowerLift(double maxTime){
        double timeElapsed;
        double runtimeValue;

        /*
         * In the case where the sensor fails or the sensor is too far away from the stones to
         * detect the skystone, we want the function to abort the effort and go to grab the closest
         * stone and finish the autonomous mode.  To do this, we track the time the algorithm runs
         * and tell it to cancel if the maxTime is exceeded.
         */
        runtimeValue = runtime.time();
        timeElapsed = runtime.time() - runtimeValue;

        // turn the linear motor on to begin lowering the lift
        robot.motorLinear.setPower(-0.400);

        while (opMode.opModeIsActive() && (!robot.touchLiftBack.isPressed()) && (timeElapsed < maxTime)){
            timeElapsed = runtime.time() - runtimeValue;
            // wait for the lift to lean all the way backward
            opMode.telemetry.addData("Status", "Run Time: " + timeElapsed);
            opMode.telemetry.update();
        }
        robot.motorLinear.setPower(0);  // shut the motor off
    }

    /*
     * Raise the 4Bar
     */
    public void raise4Bar(double maxTime){
        double timeElapsed;
        double runtimeValue;

        /*
         *
         */
        runtimeValue = runtime.time();
        timeElapsed = runtime.time() - runtimeValue;

        // turn the linear motor on to begin lowering the lift
//        robot.motor4Bar.setPower(0.300);

        while (opMode.opModeIsActive() && (!robot.touchLiftBack.isPressed()) && (timeElapsed < maxTime)){
            timeElapsed = runtime.time() - runtimeValue;
            // wait for the lift to lean all the way backward
            opMode.telemetry.addData("Status", "Run Time: " + timeElapsed);
            opMode.telemetry.update();
        }
        robot.motorLinear.setPower(0.1);  // apply enough power to keep it up
    }

    /*
     * Lower the 4Bar
     */
    public void lower4Bar(){
        /*
         *
         */
        // turn the linear motor on to begin lowering the lift
//        robot.motor4Bar.setPower(-0.1);

        opMode.sleep(200);
        robot.motorLinear.setPower(0);  // apply enough power to keep it up
    }

    /*
     * blueStonePosition determines the location of the position of the skystone (1st, 2nd, or
     * 3rd postion) on the blue side of the field.
     * It does this by using the strafe time passed to the function to determine how far the robot
     * had to strafe to locate the skystone.
     */
    public int blueStonePosition(double time){
        int position;
        if (time < 0.5){    // less than 1/2 second indicates first stone
            position = 3;
        } else if(time >= 0.8){ // greater than 1 second indicates 3rd stone Note: had to strafe to next section
            position =1;
        } else position = 2;    // else select 2nd stone
        return position;
    }

    /*
     * redStonePosition determines the location of the position of the skystone (1st, 2nd, or
     * 3rd postion) on the red side of the field.
     * It does this by using the strafe time passed to the function to determine how far the robot
     * had to strafe to locate the skystone.
     */
    public int redStonePosition(double time){
        int position;
        if (time < 0.5){    // less than 1/2 second indicates first stone
            position = 1;
        } else if(time >1){ // greater than 1 second indicates 3rd stone Note: had to strafe to next section
            position =3;
        } else position = 2;    // else select 2nd stone
        return position;
    }

    public void driveOmniVuforia(double mm, double power, double heading, double timeOut, double y) {

        heading = heading * (Math.PI / 180);

        LF = power * Math.sin(heading + (Math.PI / 4));
        RF = power * Math.cos(heading + (Math.PI / 4));
        LR = power * Math.cos(heading + (Math.PI / 4));
        RR = power * Math.sin(heading + (Math.PI / 4));

        if (LF > 1 || LF < -1) {
            LF = 0;
        }

        if (RF > 1 || RF < -1) {
            RF = 0;
        }

        if (LR > 1 || LR < -1) {
            LR = 0;
        }

        if (RR > 1 || RR < -1) {
            RR = 0;
        }
        vuforiaTracking = myVuforia.getLocation(myTrackables);
        robotX = vuforiaTracking.get(0);
        robotY = vuforiaTracking.get(1);
        robotBearing = vuforiaTracking.get(2);

        while (opMode.opModeIsActive() && robotY < y) {
            vuforiaTracking = myVuforia.getLocation(myTrackables);
            robotX = vuforiaTracking.get(0);
            robotY = vuforiaTracking.get(1);
            robotBearing = vuforiaTracking.get(2);

            robot.motorLF.setPower(LF);
            robot.motorRF.setPower(RF);
            robot.motorLR.setPower(LR);
            robot.motorRR.setPower(RR);

            myCurrentMotorPosition = robot.motorLR.getCurrentPosition();

            opMode.telemetry.addData("Status", "Run Time: " + runtime.time());
            opMode.telemetry.addData("robotX", String.valueOf(robotX));
            opMode.telemetry.addData("robotY", String.valueOf(robotY));
            opMode.telemetry.addData("robotBearing", String.valueOf(robotBearing));
            opMode.telemetry.addData("CurrentPosition", String.valueOf(myCurrentMotorPosition));
            opMode.telemetry.addData("LF", String.valueOf(LF));
            opMode.telemetry.addData("RF", String.valueOf(RF));
            opMode.telemetry.addData("LR", String.valueOf(LR));
            opMode.telemetry.addData("RR", String.valueOf(RR));
            opMode.telemetry.update();

            opMode.idle();
        }
        motorsHalt();
        opMode.requestOpModeStop();
    }

    /**
     * rotateGyro uses the Gyro sensor to control rotation of the robot.  Robot will rotate in place.
     * Note: this function does not use PID control.
     * @param power         // controls how fast to rotate
     * @param angle         // identifies what angle to rotate to
     * @param direction     // which direction to rotate - "right" or "left"
     * @param maxTime       // maximum amount of time to attempt operation
     */
    public void rotateGyro(double power, double angle, String direction, double maxTime){
        double currentZinit = robot.mrGyro.getIntegratedZValue();
        double targetZ;
        double timeElapsed;
        double runtimeValue;

        /*
         * In the case where the sensor fails or the sensor is too far away from the stones to
         * detect the skystone, we want the function to abort the effort and go to grab the closest
         * stone and finish the autonomous mode.  To do this, we track the time the algorithm runs
         * and tell it to cancel if the maxTime is exceeded.
         */
        runtimeValue = runtime.time();
        timeElapsed = runtime.time() - runtimeValue;

        switch(direction){
            case "right" :
                targetZ = currentZinit - angle;
                robot.motorLF.setPower(power);
                robot.motorLR.setPower(power);
                robot.motorRF.setPower(-power);
                robot.motorRR.setPower(-power);
                while (opMode.opModeIsActive() && (timeElapsed < maxTime) && currentZinit > targetZ){
                    currentZinit = robot.mrGyro.getIntegratedZValue();
                    timeElapsed = runtime.time() - runtimeValue;
                    opMode.telemetry.addData("Max Time : ", maxTime);
                    opMode.telemetry.addData("Elapsed Time : ", timeElapsed);
                    opMode.telemetry.addData("Gyro Value : ", currentZinit);
                    opMode.telemetry.update();
                }
                motorsHalt();
                opMode.sleep(100);
                /*
                 * Correct for overshooting the desired angle. Start by rotating opposite direction.
                 */
                currentZinit = robot.mrGyro.getIntegratedZValue();
                robot.motorLF.setPower(-0.1);
                robot.motorLR.setPower(-0.1);
                robot.motorRF.setPower(0.1);
                robot.motorRR.setPower(0.1);
                while (opMode.opModeIsActive() && (timeElapsed < maxTime) && currentZinit <= (targetZ - 2)) {
                    currentZinit = robot.mrGyro.getIntegratedZValue();
                    timeElapsed = runtime.time() - runtimeValue;
                    opMode.telemetry.addData("Max Time : ", maxTime);
                    opMode.telemetry.addData("Elapsed Time : ", timeElapsed);
                    opMode.telemetry.addData("Gyro Value : ", currentZinit);
                    opMode.telemetry.update();
                }
                motorsHalt();
                break;

            case "left" :
                targetZ = currentZinit + angle;
                robot.motorLF.setPower(-power);
                robot.motorLR.setPower(-power);
                robot.motorRF.setPower(power);
                robot.motorRR.setPower(power);
                while (opMode.opModeIsActive() && (timeElapsed < maxTime) && currentZinit <= targetZ){
                    currentZinit = robot.mrGyro.getIntegratedZValue();
                    timeElapsed = runtime.time() - runtimeValue;
                    opMode.telemetry.addData("Max Time : ", maxTime);
                    opMode.telemetry.addData("Elapsed Time : ", timeElapsed);
                    opMode.telemetry.addData("Gyro Value : ", currentZinit);
                    opMode.telemetry.update();
                }

                motorsHalt();
                opMode.sleep(100);
                /*
                 * Correct for overshooting the desired angle. Start by rotating opposite direction.
                 */
                currentZinit = robot.mrGyro.getIntegratedZValue();
                robot.motorLF.setPower(0.1);
                robot.motorLR.setPower(0.1);
                robot.motorRF.setPower(-0.1);
                robot.motorRR.setPower(-0.1);
                while (opMode.opModeIsActive() && (timeElapsed < maxTime) && currentZinit > (targetZ-2)) {
                    currentZinit = robot.mrGyro.getIntegratedZValue();
                    timeElapsed = runtime.time() - runtimeValue;
                    opMode.telemetry.addData("Max Time : ", maxTime);
                    opMode.telemetry.addData("Elapsed Time : ", timeElapsed);
                    opMode.telemetry.addData("Gyro Value : ", currentZinit);
                    opMode.telemetry.update();
                }
                motorsHalt();
                break;
        }
    }

    /**
     * rotateAndLowerLift uses the Gyro sensor to control rotation of the robot.  Robot will rotate in place.
     * Note: this function does not use PID control.
     * @param power         // controls how fast to rotate
     * @param angle         // identifies what angle to rotate to
     * @param direction     // which direction to rotate - "right" or "left"
     * @param maxTime       // maximum amount of time to attempt operation
     */
    public void rotateAndLowerLift(double power, double angle, String direction, double maxTime){
        double currentZinit = robot.mrGyro.getIntegratedZValue();
        double targetZ;
        double timeElapsed;
        double runtimeValue;

        /*
         * In the case where the sensor fails or the sensor is too far away from the stones to
         * detect the skystone, we want the function to abort the effort and go to grab the closest
         * stone and finish the autonomous mode.  To do this, we track the time the algorithm runs
         * and tell it to cancel if the maxTime is exceeded.
         */
        runtimeValue = runtime.time();
        timeElapsed = runtime.time() - runtimeValue;

        // turn the linear motor on to begin lowering the lift
        robot.motorLinear.setPower(-0.400);

        switch(direction){
            case "right" :
                targetZ = currentZinit - angle;
                robot.motorLF.setPower(power);
                robot.motorLR.setPower(power);
                robot.motorRF.setPower(-power);
                robot.motorRR.setPower(-power);
                while (opMode.opModeIsActive() && (timeElapsed < maxTime) && currentZinit > targetZ){
                    // Shut the linear slide off if the lift has been lowered
                    if (robot.touchLiftBack.isPressed()) robot.motorLinear.setPower(0);
                    currentZinit = robot.mrGyro.getIntegratedZValue();
                    timeElapsed = runtime.time() - runtimeValue;
                    opMode.telemetry.addData("Max Time : ", maxTime);
                    opMode.telemetry.addData("Elapsed Time : ", timeElapsed);
                    opMode.telemetry.addData("Gyro Value : ", currentZinit);
                    opMode.telemetry.update();
                }
                motorsHalt();
                robot.motorLinear.setPower(0);      // shut the linear motor off
                /*
                 * Correct for overshooting the desired angle. Start by rotating opposite direction.
                 */
                currentZinit = robot.mrGyro.getIntegratedZValue();
                robot.motorLF.setPower(-0.1);
                robot.motorLR.setPower(-0.1);
                robot.motorRF.setPower(0.1);
                robot.motorRR.setPower(0.1);
                while (opMode.opModeIsActive() && (timeElapsed < maxTime) && currentZinit <= (targetZ - 2)) {
                    // Shut the linear slide off if the lift has been lowered
                    if (robot.touchLiftBack.isPressed()) robot.motorLinear.setPower(0);
                    currentZinit = robot.mrGyro.getIntegratedZValue();
                    timeElapsed = runtime.time() - runtimeValue;
                    opMode.telemetry.addData("Max Time : ", maxTime);
                    opMode.telemetry.addData("Elapsed Time : ", timeElapsed);
                    opMode.telemetry.addData("Gyro Value : ", currentZinit);
                    opMode.telemetry.update();
                }
                motorsHalt();
                break;

            case "left" :
                targetZ = currentZinit + angle;
                robot.motorLF.setPower(-power);
                robot.motorLR.setPower(-power);
                robot.motorRF.setPower(power);
                robot.motorRR.setPower(power);
                while (opMode.opModeIsActive() && (timeElapsed < maxTime) && currentZinit <= targetZ){
                    // Shut the linear slide off if the lift has been lowered
                    if (robot.touchLiftBack.isPressed()) robot.motorLinear.setPower(0);
                    currentZinit = robot.mrGyro.getIntegratedZValue();
                    timeElapsed = runtime.time() - runtimeValue;
                    opMode.telemetry.addData("Max Time : ", maxTime);
                    opMode.telemetry.addData("Elapsed Time : ", timeElapsed);
                    opMode.telemetry.addData("Gyro Value : ", currentZinit);
                    opMode.telemetry.update();
                }

                motorsHalt();
                robot.motorLinear.setPower(0);      // shut the linear motor off

                /*
                 * Correct for overshooting the desired angle. Start by rotating opposite direction.
                 */
                currentZinit = robot.mrGyro.getIntegratedZValue();
                robot.motorLF.setPower(0.1);
                robot.motorLR.setPower(0.1);
                robot.motorRF.setPower(-0.1);
                robot.motorRR.setPower(-0.1);
                while (opMode.opModeIsActive() && (timeElapsed < maxTime) && currentZinit > (targetZ-2)) {
                    // Shut the linear slide off if the lift has been lowered
                    if (robot.touchLiftBack.isPressed()) robot.motorLinear.setPower(0);
                    currentZinit = robot.mrGyro.getIntegratedZValue();
                    timeElapsed = runtime.time() - runtimeValue;
                    opMode.telemetry.addData("Max Time : ", maxTime);
                    opMode.telemetry.addData("Elapsed Time : ", timeElapsed);
                    opMode.telemetry.addData("Gyro Value : ", currentZinit);
                    opMode.telemetry.update();
                }
                motorsHalt();
                robot.motorLinear.setPower(0);
                break;
        }
    }

    /**
     * rotateAndRaiseLift uses the Gyro sensor to control rotation of the robot.  Robot will rotate in place.
     * Note: this function does not use PID control.
     * @param power         // controls how fast to rotate
     * @param angle         // identifies what angle to rotate to
     * @param direction     // which direction to rotate - "right" or "left"
     * @param maxTime       // maximum amount of time to attempt operation
     */
    public void rotateAndRaiseLift(double power, double angle, String direction, double maxTime){
        double currentZinit = robot.mrGyro.getIntegratedZValue();
        double targetZ;
        double timeElapsed;
        double runtimeValue;

        /*
         * In the case where the sensor fails or the sensor is too far away from the stones to
         * detect the skystone, we want the function to abort the effort and go to grab the closest
         * stone and finish the autonomous mode.  To do this, we track the time the algorithm runs
         * and tell it to cancel if the maxTime is exceeded.
         */
        runtimeValue = runtime.time();
        timeElapsed = runtime.time() - runtimeValue;

        // turn the linear motor on to begin lowering the lift
        robot.motorLinear.setPower(0.400);

        while (opMode.opModeIsActive() && (!robot.touchLiftForward.isPressed()) && (timeElapsed < maxTime)){
            timeElapsed = runtime.time() - runtimeValue;
            // wait for the lift to lean all the way backward
            opMode.telemetry.addData("Status", "Run Time: " + timeElapsed);
            opMode.telemetry.update();
        }
        robot.motorLinear.setPower(0);  // shut the motor off


        switch(direction){
            case "right" :
                targetZ = currentZinit - angle;
                robot.motorLF.setPower(power);
                robot.motorLR.setPower(power);
                robot.motorRF.setPower(-power);
                robot.motorRR.setPower(-power);
                while (opMode.opModeIsActive() && (timeElapsed < maxTime) && currentZinit > targetZ){
                    // Shut the linear slide off if the lift has been lowered
                    if (robot.touchLiftBack.isPressed()) robot.motorLinear.setPower(0);
                    currentZinit = robot.mrGyro.getIntegratedZValue();
                    timeElapsed = runtime.time() - runtimeValue;
                    opMode.telemetry.addData("Max Time : ", maxTime);
                    opMode.telemetry.addData("Elapsed Time : ", timeElapsed);
                    opMode.telemetry.addData("Gyro Value : ", currentZinit);
                    opMode.telemetry.update();
                }
                motorsHalt();
                opMode.sleep(100);
                /**
                 * Correct for overshooting the desired angle. Start by rotating opposite direction.
                 */
                currentZinit = robot.mrGyro.getIntegratedZValue();
                robot.motorLF.setPower(-0.1);
                robot.motorLR.setPower(-0.1);
                robot.motorRF.setPower(0.1);
                robot.motorRR.setPower(0.1);
                while (opMode.opModeIsActive() && (timeElapsed < maxTime) && currentZinit <= (targetZ - 2)) {
                    // Shut the linear slide off if the lift has been lowered
                    if (robot.touchLiftBack.isPressed()) robot.motorLinear.setPower(0);
                    currentZinit = robot.mrGyro.getIntegratedZValue();
                    timeElapsed = runtime.time() - runtimeValue;
                    opMode.telemetry.addData("Max Time : ", maxTime);
                    opMode.telemetry.addData("Elapsed Time : ", timeElapsed);
                    opMode.telemetry.addData("Gyro Value : ", currentZinit);
                    opMode.telemetry.update();
                }
                motorsHalt();
                break;

            case "left" :
                targetZ = currentZinit + angle;
                robot.motorLF.setPower(-power);
                robot.motorLR.setPower(-power);
                robot.motorRF.setPower(power);
                robot.motorRR.setPower(power);
                while (opMode.opModeIsActive() && (timeElapsed < maxTime) && currentZinit <= targetZ){
                    // Shut the linear slide off if the lift has been lowered
                    if (robot.touchLiftBack.isPressed()) robot.motorLinear.setPower(0);
                    currentZinit = robot.mrGyro.getIntegratedZValue();
                    timeElapsed = runtime.time() - runtimeValue;
                    opMode.telemetry.addData("Max Time : ", maxTime);
                    opMode.telemetry.addData("Elapsed Time : ", timeElapsed);
                    opMode.telemetry.addData("Gyro Value : ", currentZinit);
                    opMode.telemetry.update();
                }

                motorsHalt();
                opMode.sleep(100);
                /**
                 * Correct for overshooting the desired angle. Start by rotating opposite direction.
                 */
                currentZinit = robot.mrGyro.getIntegratedZValue();
                robot.motorLF.setPower(0.1);
                robot.motorLR.setPower(0.1);
                robot.motorRF.setPower(-0.1);
                robot.motorRR.setPower(-0.1);
                while (opMode.opModeIsActive() && (timeElapsed < maxTime) && currentZinit > (targetZ-2)) {
                    // Shut the linear slide off if the lift has been lowered
                    if (robot.touchLiftBack.isPressed()) robot.motorLinear.setPower(0);
                    currentZinit = robot.mrGyro.getIntegratedZValue();
                    timeElapsed = runtime.time() - runtimeValue;
                    opMode.telemetry.addData("Max Time : ", maxTime);
                    opMode.telemetry.addData("Elapsed Time : ", timeElapsed);
                    opMode.telemetry.addData("Gyro Value : ", currentZinit);
                    opMode.telemetry.update();
                }
                motorsHalt();
                robot.motorLinear.setPower(0);
                break;
        }
    }

    public void motorsHalt() {
        robot.motorLF.setPower(0);
        robot.motorRF.setPower(0);
        robot.motorLR.setPower(0);
        robot.motorRR.setPower(0);
    }
}