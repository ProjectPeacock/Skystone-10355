package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import java.util.List;


/*
 * FTC Team 10355 driveMecanum Class for 2019-2020 Skystone season.
 */

public class DriveMecanumV3 {
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


    public DriveMecanumV3(HardwareProfile myRobot, LinearOpMode myOpMode, skystoneVuforia thisVuforia,
                          List<VuforiaTrackable> trackableList) {
        robot = myRobot;
        opMode = myOpMode;
        myVuforia = thisVuforia;
        myTrackables = trackableList;
    }

    public void translateVuforiaNavY(double power, double heading, double timeOut, double y) {
        double initZ = getZAngle();

        vuforiaTracking = myVuforia.getLocation(myTrackables);
        robotX = vuforiaTracking.get(0);
        robotY = vuforiaTracking.get(1);
        robotBearing = vuforiaTracking.get(2);

        moveRobot(power, heading);

        while (opMode.opModeIsActive() && runtime.time() < timeOut && robotY < y) {
            opMode.telemetry.addData("if (y > startingY)", "");
            checkZOrientation(heading, initZ);

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

    public double getZAngle() {
        return robot.imu.getAngularOrientation().firstAngle;
    }
    public void translateVuforiaNavX(double power, double heading, double timeOut, double x) {
        double initZ = getZAngle();
        double startingX;

        heading = heading * (Math.PI / 180);

        vuforiaTracking = myVuforia.getLocation(myTrackables);
        robotX = vuforiaTracking.get(0);
        robotY = vuforiaTracking.get(1);
        robotBearing = vuforiaTracking.get(2);

        startingX = robotX;

        if (x < startingX) {
            moveRobot(power,heading);
            while (opMode.opModeIsActive() && runtime.time() < timeOut && robotX > x) {
                checkZOrientation(heading, initZ);

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
    }

    public void translateTime(double power, double heading, double timeOut) {
        double initZ = getZAngle();
        double timeElapsed;
        double runtimeValue;

        runtimeValue = runtime.time();
        timeElapsed = runtimeValue - runtime.time();
        moveRobot(power,heading);
        while (opMode.opModeIsActive() && timeElapsed < timeOut) {
            checkZOrientation(heading, initZ);

            timeElapsed = runtime.time() - runtimeValue;
            opMode.telemetry.addData("Status", "Run Time: " + runtime.time());
            opMode.telemetry.addData("Status", "Elapsed Time: " + timeElapsed);
            //   opMode.telemetry.addData("Gyro Value", String.valueOf(currentZint));
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

    public void translateSkystone(double power, double heading, double alphaColor, double maxTime) {
        double initZ = getZAngle();
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
        moveRobot(power,heading);
        while (opMode.opModeIsActive() && (robot.colorSensorRevStone.red() > alphaColor) && (timeElapsed < maxTime)) {
            checkZOrientation(heading, initZ);

            timeElapsed = runtime.time() - runtimeValue;
            opMode.telemetry.addData("Status", "Run Time: " + runtime.time());
            opMode.telemetry.addData("Status", "Elapsed Time: " + timeElapsed);
            opMode.telemetry.addData("Color sensor Value", String.valueOf(robot.colorSensorRevStone.red()));
            //   opMode.telemetry.addData("Gyro Value", String.valueOf(currentZint));
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
     *
     * @param power    // controls speed at which the robot should move
     * @param heading  // direction to head (will strafe to the location
     * @param distance // distance to the wall that the robot is trying to acheive
     * @param maxTime  // maximum time that the function should run - exits at maxTime
     */
    public void robotCorrect(double power, double heading, double distance, double maxTime) {
        double initZ = getZAngle();
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
        while (opMode.opModeIsActive() && active) {
            if (currentRangeB < 60 && currentRangeF < 18) {
                translateTime(0.6, 90, 1);

                while (currentRangeR < 5) {
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
     *
     * @param power    // controls speed at which the robot should move
     * @param heading  // direction to head (will strafe to the location
     * @param distance // distance to the wall that the robot is trying to acheive
     * @param maxTime  // maximum time that the function should run - exits at maxTime
     */

    public void translateFromWall(double power, double heading, double distance, double maxTime) {
        double initZ = getZAngle();
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

        currentRange = robot.wallRangeSensor.getDistance(DistanceUnit.CM);
        if (currentRange > distance) {
            active = false;
        } else{
            moveRobot(power,heading);
        }

        while (opMode.opModeIsActive() && active & (timeElapsed < maxTime)) {
            checkZOrientation(heading, initZ);

            // check to see if the distance traveled is less than the range specification
            currentRange = robot.wallRangeSensor.getDistance(DistanceUnit.CM);
            if (currentRange > distance) active = false;

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


    /**
     * translateToWall will use the range sensor to measure distance to the wall and will drive
     * to the specified distance.
     *
     * @param power    // controls speed at which the robot should move
     * @param heading  // direction to head (will strafe to the location
     * @param distance // distance to the wall that the robot is trying to acheive
     * @param maxTime  // maximum time that the function should run - exits at maxTime
     */
    public void translateToWall(double power, double heading, double distance, double maxTime) {
        double initZ = getZAngle();
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

        currentRange = robot.wallRangeSensor.getDistance(DistanceUnit.CM);
        if (currentRange < distance) {
            active = false;
        } else {
            moveRobot(power,heading);
        }

        while (opMode.opModeIsActive() && active & (timeElapsed < maxTime)) {
            checkZOrientation(heading, initZ);

            // check to see if the distance traveled is less than the range specification
            currentRange = robot.wallRangeSensor.getDistance(DistanceUnit.CM);
            if (currentRange < distance) active = false;

            timeElapsed = runtime.time() - runtimeValue;
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
    public void raiseLift(double maxTime) {
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

        while (opMode.opModeIsActive() && (!robot.touchLiftForward.isPressed()) && (timeElapsed < maxTime)) {
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
    public void lowerLift(double maxTime) {
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

        while (opMode.opModeIsActive() && (!robot.touchLiftBack.isPressed()) && (timeElapsed < maxTime)) {
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
    public void raise4Bar(double maxTime) {
        double timeElapsed;
        double runtimeValue;

        /*
         *
         */
        runtimeValue = runtime.time();
        timeElapsed = runtime.time() - runtimeValue;

        // turn the linear motor on to begin lowering the lift
//        robot.motor4Bar.setPower(0.300);

        while (opMode.opModeIsActive() && (!robot.touchLiftBack.isPressed()) && (timeElapsed < maxTime)) {
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
    public void lower4Bar() {
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
    public int blueStonePosition(double time) {
        int position;
        if (time < 0.5) {    // less than 1/2 second indicates first stone
            position = 3;
        } else if (time >= 0.8) { // greater than 1 second indicates 3rd stone Note: had to strafe to next section
            position = 1;
        } else position = 2;    // else select 2nd stone
        return position;
    }

    /*
     * redStonePosition determines the location of the position of the skystone (1st, 2nd, or
     * 3rd postion) on the red side of the field.
     * It does this by using the strafe time passed to the function to determine how far the robot
     * had to strafe to locate the skystone.
     */
    public int redStonePosition(double time) {
        int position;
        if (time < 0.5) {    // less than 1/2 second indicates first stone
            position = 1;
        } else if (time > 1) { // greater than 1 second indicates 3rd stone Note: had to strafe to next section
            position = 3;
        } else position = 2;    // else select 2nd stone
        return position;
    }

    public void driveOmniVuforia(double mm, double power, double heading,
                                 double timeOut, double y) {

        heading = heading * (Math.PI / 180);

        LF = power * Math.sin(heading + (Math.PI / 4));
        RF = power * Math.cos(heading + (Math.PI / 4));
        LR = power * Math.cos(heading + (Math.PI / 4));
        RR = power * Math.sin(heading + (Math.PI / 4));

        // smooth power settings to -1 to 1 range limit
        double max = Math.max(Math.max(Math.max(Math.abs(LF), Math.abs(RF)), Math.abs(LR)), Math.abs(RR));

        if (max > 1) {
            LF = LF / max;
            RF = RF / max;
            LR = LR / max;
            RR = RR / max;
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
     *
     * @param power     // controls how fast to rotate
     * @param angle     // identifies what angle to rotate to
     * @param direction // which direction to rotate - "right" or "left"
     * @param maxTime   // maximum amount of time to attempt operation
     */
    public void rotateGyro(double power, double angle, String direction, double maxTime) {
        double currentZinit = getZAngle();
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

        switch (direction) {
            case "right":
                targetZ = currentZinit - angle;
                robot.motorLF.setPower(power);
                robot.motorLR.setPower(power);
                robot.motorRF.setPower(-power);
                robot.motorRR.setPower(-power);
                while (opMode.opModeIsActive() && (timeElapsed < maxTime) && currentZinit > targetZ) {
                    currentZinit = getZAngle();
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
                currentZinit = getZAngle();
                robot.motorLF.setPower(-0.1);
                robot.motorLR.setPower(-0.1);
                robot.motorRF.setPower(0.1);
                robot.motorRR.setPower(0.1);
                while (opMode.opModeIsActive() && (timeElapsed < maxTime) && currentZinit <= (targetZ - 2)) {
                    currentZinit = getZAngle();
                    timeElapsed = runtime.time() - runtimeValue;
                    opMode.telemetry.addData("Max Time : ", maxTime);
                    opMode.telemetry.addData("Elapsed Time : ", timeElapsed);
                    opMode.telemetry.addData("Gyro Value : ", currentZinit);
                    opMode.telemetry.update();
                }
                motorsHalt();
                break;

            case "left":
                targetZ = currentZinit + angle;
                robot.motorLF.setPower(-power);
                robot.motorLR.setPower(-power);
                robot.motorRF.setPower(power);
                robot.motorRR.setPower(power);
                while (opMode.opModeIsActive() && (timeElapsed < maxTime) && currentZinit <= targetZ) {
                    currentZinit = getZAngle();
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
                currentZinit = getZAngle();
                robot.motorLF.setPower(0.1);
                robot.motorLR.setPower(0.1);
                robot.motorRF.setPower(-0.1);
                robot.motorRR.setPower(-0.1);
                while (opMode.opModeIsActive() && (timeElapsed < maxTime) && currentZinit > (targetZ - 2)) {
                    currentZinit = getZAngle();
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
     *
     * @param power     // controls how fast to rotate
     * @param angle     // identifies what angle to rotate to
     * @param direction // which direction to rotate - "right" or "left"
     * @param maxTime   // maximum amount of time to attempt operation
     */
    public void rotateAndLowerLift(double power, double angle, String direction, double maxTime) {
        double currentZinit = getZAngle();
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

        switch (direction) {
            case "right":
                targetZ = currentZinit - angle;
                robot.motorLF.setPower(power);
                robot.motorLR.setPower(power);
                robot.motorRF.setPower(-power);
                robot.motorRR.setPower(-power);
                while (opMode.opModeIsActive() && (timeElapsed < maxTime) && currentZinit > targetZ) {
                    // Shut the linear slide off if the lift has been lowered
                    if (robot.touchLiftBack.isPressed()) robot.motorLinear.setPower(0);
                    currentZinit = getZAngle();
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
                currentZinit = getZAngle();
                robot.motorLF.setPower(-0.1);
                robot.motorLR.setPower(-0.1);
                robot.motorRF.setPower(0.1);
                robot.motorRR.setPower(0.1);
                while (opMode.opModeIsActive() && (timeElapsed < maxTime) && currentZinit <= (targetZ - 2)) {
                    // Shut the linear slide off if the lift has been lowered
                    if (robot.touchLiftBack.isPressed()) robot.motorLinear.setPower(0);
                    currentZinit = getZAngle();
                    timeElapsed = runtime.time() - runtimeValue;
                    opMode.telemetry.addData("Max Time : ", maxTime);
                    opMode.telemetry.addData("Elapsed Time : ", timeElapsed);
                    opMode.telemetry.addData("Gyro Value : ", currentZinit);
                    opMode.telemetry.update();
                }
                motorsHalt();
                break;

            case "left":
                targetZ = currentZinit + angle;
                robot.motorLF.setPower(-power);
                robot.motorLR.setPower(-power);
                robot.motorRF.setPower(power);
                robot.motorRR.setPower(power);
                while (opMode.opModeIsActive() && (timeElapsed < maxTime) && currentZinit <= targetZ) {
                    // Shut the linear slide off if the lift has been lowered
                    if (robot.touchLiftBack.isPressed()) robot.motorLinear.setPower(0);
                    currentZinit = getZAngle();
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
                currentZinit = getZAngle();
                robot.motorLF.setPower(0.1);
                robot.motorLR.setPower(0.1);
                robot.motorRF.setPower(-0.1);
                robot.motorRR.setPower(-0.1);
                while (opMode.opModeIsActive() && (timeElapsed < maxTime) && currentZinit > (targetZ - 2)) {
                    // Shut the linear slide off if the lift has been lowered
                    if (robot.touchLiftBack.isPressed()) robot.motorLinear.setPower(0);
                    currentZinit = getZAngle();
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
     *
     * @param power     // controls how fast to rotate
     * @param angle     // identifies what angle to rotate to
     * @param direction // which direction to rotate - "right" or "left"
     * @param maxTime   // maximum amount of time to attempt operation
     */
    public void rotateAndRaiseLift(double power, double angle, String direction, double maxTime) {
        double currentZinit = getZAngle();
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

        while (opMode.opModeIsActive() && (!robot.touchLiftForward.isPressed()) && (timeElapsed < maxTime)) {
            timeElapsed = runtime.time() - runtimeValue;
            // wait for the lift to lean all the way backward
            opMode.telemetry.addData("Status", "Run Time: " + timeElapsed);
            opMode.telemetry.update();
        }
        robot.motorLinear.setPower(0);  // shut the motor off

        switch (direction) {
            case "right":
                targetZ = currentZinit - angle;
                robot.motorLF.setPower(power);
                robot.motorLR.setPower(power);
                robot.motorRF.setPower(-power);
                robot.motorRR.setPower(-power);
                while (opMode.opModeIsActive() && (timeElapsed < maxTime) && currentZinit > targetZ) {
                    // Shut the linear slide off if the lift has been lowered
                    if (robot.touchLiftBack.isPressed()) robot.motorLinear.setPower(0);
                    currentZinit = getZAngle();
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
                currentZinit = getZAngle();
                robot.motorLF.setPower(-0.1);
                robot.motorLR.setPower(-0.1);
                robot.motorRF.setPower(0.1);
                robot.motorRR.setPower(0.1);
                while (opMode.opModeIsActive() && (timeElapsed < maxTime) && currentZinit <= (targetZ - 2)) {
                    // Shut the linear slide off if the lift has been lowered
                    if (robot.touchLiftBack.isPressed()) robot.motorLinear.setPower(0);
                    currentZinit = getZAngle();
                    timeElapsed = runtime.time() - runtimeValue;
                    opMode.telemetry.addData("Max Time : ", maxTime);
                    opMode.telemetry.addData("Elapsed Time : ", timeElapsed);
                    opMode.telemetry.addData("Gyro Value : ", currentZinit);
                    opMode.telemetry.update();
                }
                motorsHalt();
                break;

            case "left":
                targetZ = currentZinit + angle;
                robot.motorLF.setPower(-power);
                robot.motorLR.setPower(-power);
                robot.motorRF.setPower(power);
                robot.motorRR.setPower(power);
                while (opMode.opModeIsActive() && (timeElapsed < maxTime) && currentZinit <= targetZ) {
                    // Shut the linear slide off if the lift has been lowered
                    if (robot.touchLiftBack.isPressed()) robot.motorLinear.setPower(0);
                    currentZinit = getZAngle();
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
                currentZinit = getZAngle();
                robot.motorLF.setPower(0.1);
                robot.motorLR.setPower(0.1);
                robot.motorRF.setPower(-0.1);
                robot.motorRR.setPower(-0.1);
                while (opMode.opModeIsActive() && (timeElapsed < maxTime) && currentZinit > (targetZ - 2)) {
                    // Shut the linear slide off if the lift has been lowered
                    if (robot.touchLiftBack.isPressed()) robot.motorLinear.setPower(0);
                    currentZinit = getZAngle();
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

    public boolean rotatingZ(double targetZ, String direction) {
        double currentZinit = getZAngle();
        switch (direction) {
            case "right":
                if (currentZinit >= targetZ) {
                    return false;
                } else {
                    return true;
                }
            case "left":
                if (currentZinit <= targetZ) {
                    return false;
                } else {
                    return true;
                }
            default:
                return false; // just to prevent an infinite loop
        }
    }

    public boolean correctingZ(double targetZ, String direction) {
        double currentZinit = getZAngle();
        switch (direction) {
            case "right":
                if (currentZinit <= (targetZ - 2)) {
                    return false;
                } else {
                    return true;
                }
            case "left":
                if (currentZinit >= (targetZ - 2)) {
                    return false;
                } else {
                    return true;
                }
            default:
                return false; // just to prevent an infinite loop
        }
    }

    public void rotateGyroV2(double power, double angle, String direction, double maxTime) {
        double currentZinit = getZAngle();
        double targetZ = 0.0;
        double timeElapsed;
        double runtimeValue;
        double leftCorrection = 0.0;
        double rightCorrection = 0.0;

        /*
         * In the case where the sensor fails or the sensor is too far away from the stones to
         * detect the skystone, we want the function to abort the effort and go to grab the closest
         * stone and finish the autonomous mode.  To do this, we track the time the algorithm runs
         * and tell it to cancel if the maxTime is exceeded.
         */
        runtimeValue = runtime.time();
        timeElapsed = runtime.time() - runtimeValue;

        switch (direction) {
            case "right":
                targetZ = currentZinit - angle;
                robot.motorLF.setPower(power);
                robot.motorLR.setPower(power);
                robot.motorRF.setPower(-power);
                robot.motorRR.setPower(-power);
                leftCorrection = -0.1;
                rightCorrection = 0.1;
                break;
            case "left":
                targetZ = currentZinit + angle;
                robot.motorLF.setPower(-power);
                robot.motorLR.setPower(-power);
                robot.motorRF.setPower(power);
                robot.motorRR.setPower(power);
                leftCorrection = 0.1;
                rightCorrection = -0.1;
                break;
        }

        while (opMode.opModeIsActive() && (timeElapsed < maxTime) && rotatingZ(targetZ, direction)) {
            timeElapsed = runtime.time() - runtimeValue;
            opMode.telemetry.addData("Max Time : ", maxTime);
            opMode.telemetry.addData("Elapsed Time : ", timeElapsed);
            opMode.telemetry.update();
        }
        motorsHalt();
        opMode.sleep(100);

        robot.motorLF.setPower(leftCorrection);
        robot.motorLR.setPower(leftCorrection);
        robot.motorRF.setPower(rightCorrection);
        robot.motorRR.setPower(rightCorrection);

        while (opMode.opModeIsActive() && (timeElapsed < maxTime) && correctingZ(targetZ, direction)) {
            timeElapsed = runtime.time() - runtimeValue;
            opMode.telemetry.addData("Max Time : ", maxTime);
            opMode.telemetry.addData("Elapsed Time : ", timeElapsed);
            opMode.telemetry.update();
        }

    }

    public void motorsHalt() {
        robot.motorLF.setPower(0);
        robot.motorRF.setPower(0);
        robot.motorLR.setPower(0);
        robot.motorRR.setPower(0);
    }

    /*
     * Function to move robot using a specified poer, heading, and speed value.
     * gyro is used to determine if robot is drifting off course
     */

    public void moveRobot(double power, double heading) {
        heading = heading * (Math.PI / 180);

        LF = power * Math.sin(heading + (Math.PI / 4));
        RF = power * Math.cos(heading + (Math.PI / 4));
        LR = power * Math.cos(heading + (Math.PI / 4));
        RR = power * Math.sin(heading + (Math.PI / 4));
        // smooth power settings to -1 to 1 range limit
        double max = Math.max(Math.max(Math.max(Math.abs(LF), Math.abs(RF)), Math.abs(LR)), Math.abs(RR));

        if (max > 1) {
            LF = LF / max;
            RF = RF / max;
            LR = LR / max;
            RR = RR / max;
        }

        robot.motorLF.setPower(LF);
        robot.motorRF.setPower(RF);
        robot.motorLR.setPower(LR);
        robot.motorRR.setPower(RR);

    }

    public void checkZOrientation(double heading, double initZ) {
        double currentZint;
        currentZint = getZAngle();

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
           // smooth power settings to -1 to 1 range limit
        double max = Math.max(Math.max(Math.max(Math.abs(LF), Math.abs(RF)), Math.abs(LR)), Math.abs(RR));

        if (max > 1) {
            LF = LF / max;
            RF = RF / max;
            LR = LR / max;
            RR = RR / max;
        }

        robot.motorLF.setPower(LF);
        robot.motorRF.setPower(RF);
        robot.motorLR.setPower(LR);
        robot.motorRR.setPower(RR);

        myCurrentMotorPosition = robot.motorLR.getCurrentPosition();
        }
    }
}