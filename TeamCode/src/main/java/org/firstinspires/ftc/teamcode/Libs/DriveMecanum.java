package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import java.util.List;

/**
 * FTC Team 10355 driveMecanum Class for 2019-2020 Skystone season.
 */

public class DriveMecanum {
    private double myCurrentMotorPosition;
    private double myTargetPosition;
    private double LF, RF, LR, RR;
    private List<VuforiaTrackable> myTrackables;
    private HardwareProfile robot = null;
    private LinearOpMode opMode = null;
    private ElapsedTime runtime = new ElapsedTime();
    private skystoneVuforia myVuforia = null;
    private List<Double> vuforiaTracking;
    private double robotX;          // The robot's X position from VuforiaLib
    private double robotY;  // The robot's Y position from VuforiaLib
    private double robotBearing;    //Bearing to, i.e. the bearing you need to stear toward


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

            robot.motorLF.setPower(LF);
            robot.motorRF.setPower(RF);
            robot.motorLR.setPower(LR);
            robot.motorRR.setPower(RR);

            myCurrentMotorPosition = robot.motorLR.getCurrentPosition();

            vuforiaTracking = myVuforia.getLocation(myTrackables);
            robotX = vuforiaTracking.get(0);
            robotY = vuforiaTracking.get(1);
            robotBearing = vuforiaTracking.get(2);

            opMode.telemetry.addData("Status", "Run Time: " + String.valueOf(runtime.time()));
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
        double changeSpeed = 0;
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

            opMode.telemetry.addData("Status", "Run Time: " + String.valueOf(runtime.time()));
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
        double changeSpeed = 0;
        double initZ = robot.mrGyro.getIntegratedZValue();
        double currentZint;
        double timeElapsed;
        double runtimeValue;

        runtimeValue = runtime.time();
        timeElapsed = runtimeValue - runtime.time();
        heading = heading * (Math.PI / 180);

        while (opMode.opModeIsActive() && timeElapsed < timeOut) {

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
            robot.motorLF.setPower(LF);
            robot.motorRF.setPower(RF);
            robot.motorLR.setPower(LR);
            robot.motorRR.setPower(RR);

            myCurrentMotorPosition = robot.motorLR.getCurrentPosition();

            timeElapsed = runtime.time() - runtimeValue;
            opMode.telemetry.addData("Status", "Run Time: " + String.valueOf(runtime.time()));
            opMode.telemetry.addData("Status", "Elapsed Time: " + String.valueOf(timeElapsed));
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
     * Robot will drive in the heading provided by the function call.  The sensor identified should
     * be the sensor on the side of the robot that the robot is heading to.  The robot will stop
     * when the robot is within the distance provided in the parameters.
     * @param sensor
     * @param power
     * @param heading
     * @param distance
     */

    public void translateRange(String sensor, double power, double heading, double distance) {
        double changeSpeed = 0;
        double initZ = robot.mrGyro.getIntegratedZValue();
        double currentZint;
        double currentRange = 0;
        boolean active = true;

        if (sensor == "front"){
            currentRange = robot.rangeSensorFront.rawUltrasonic();
        } else if(sensor == "rear"){
            currentRange = robot.rangeSensorRear.rawUltrasonic();
        } else if(sensor == "left") {
            currentRange = robot.rangeSensorLeft.rawUltrasonic();
        } else if (sensor == "right"){
            currentRange = robot.rangeSensorRight.rawUltrasonic();
        } else active = false;      // a proper range sensor was not identified. Exit routine.

        heading = heading * (Math.PI / 180);

        while (opMode.opModeIsActive() && active) {

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
            robot.motorLF.setPower(LF);
            robot.motorRF.setPower(RF);
            robot.motorLR.setPower(LR);
            robot.motorRR.setPower(RR);

            myCurrentMotorPosition = robot.motorLR.getCurrentPosition();

            if (sensor == "front"){
                currentRange = robot.rangeSensorFront.rawUltrasonic();
            } else if(sensor == "rear"){
                currentRange = robot.rangeSensorRear.rawUltrasonic();
            } else if(sensor == "left") {
                currentRange = robot.rangeSensorLeft.rawUltrasonic();
            } else if (sensor == "right"){
                currentRange = robot.rangeSensorRight.rawUltrasonic();
            } else active = false;      // a proper range sensor was not identified. Exit routine.

            // check to see if the distance traveled is less than the range specification
            if (currentRange < distance){
                active = false;
            }
            opMode.telemetry.addData("Status", "Run Time: " + String.valueOf(runtime.time()));
            opMode.telemetry.addData("LF", String.valueOf(LF));
            opMode.telemetry.addData("RF", String.valueOf(RF));
            opMode.telemetry.addData("LR", String.valueOf(LR));
            opMode.telemetry.addData("RR", String.valueOf(RR));
            opMode.telemetry.update();

            opMode.idle();
        }
        motorsHalt();
    }

    public void driveOmniVuforia(double mm, double power, double heading, double changeSpeed,
                                 double timeOut, double y) {

        heading = heading * (Math.PI / 180);

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

            opMode.telemetry.addData("Status", "Run Time: " + String.valueOf(runtime.time()));
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

    public void motorsHalt() {
        robot.motorLF.setPower(0);
        robot.motorRF.setPower(0);
        robot.motorLR.setPower(0);
        robot.motorRR.setPower(0);
    }
}
