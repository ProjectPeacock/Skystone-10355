//package org.firstinspires.ftc.teamcode.HardwareProfiles;
package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
//import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;

/**
 * This is the hardware definition for the Hardware test/reference platform.  This class should be
 * instantiated in the opmode to allow access to the hardware.
 * <p>
 * Example:
 * <p>
 * private HardwareProfile robot = new HardwareProfile();
 */

public class HardwareProfile {
    //Wheel Setup
    public final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    public final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    public final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    public final double COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            ((WHEEL_DIAMETER_INCHES * 25.41) * Math.PI);

    /* Public OpMode members. */
    public DcMotor motorLF = null;              //Declare the motor
    public DcMotor motorRF = null;              //Declare the motor
    public DcMotor motorLR = null;              //Declare the motor
    public DcMotor motorRR = null;              //Declare the motor
    public DcMotor motorFeeder = null;          //Declare the motor
    public DcMotor motorShooter = null;         //Declare the motor
    public OpticalDistanceSensor ods;           //Declare the sensor
    public ColorSensor colorSensorRight;        //Declare the Color Sensor
    public ColorSensor colorSensorLeft;         //Declare the Color Sensor
    public TouchSensor touchSensor;             //Declare the Touch Sensor
    public GyroSensor sensorGyro;               //Declare the GyroNew sensor
    public ModernRoboticsI2cGyro mrGyro;        //Declare the MR GyroNew
    public Range rangeSensorLeft;               //Declare the left range sensor
    public Range rangeSensorRight;              //Declare the right range sensor
    public Servo servoInTake;                   //Declare the servo
    public Servo servoBallBumper;                      //Declare the servo
    public Servo servoFeeder;                      //Declare the servo

    /* I2C Range Sensor members*/
    I2cDevice rangeLeft;
    I2cDevice rangeRight;
    I2cDeviceSynch rangeLeftReader;
    I2cDeviceSynch rangeRightReader;

    /* Constructor */
    public HardwareProfile() {

    }

    /**
     * Map all the robots hardware
     *
     * @param ahwMap Input hardwaremap
     */
    public void init(HardwareMap ahwMap) {
        String platform = "mecanum";
        // Save reference to Hardware map
        HardwareMap hwMap;
        hwMap = ahwMap;

        if (platform.equals("mecanum")) {
            //Define the sensors
            ods = hwMap.opticalDistanceSensor.get("ODS");  //Map the sensor to the hardware
            I2cAddr i2CAddressColorRight = I2cAddr.create8bit(0x3c);
            I2cAddr i2CAddressColorLeft = I2cAddr.create8bit(0x4c);
            I2cAddr i2CAddressRangeRight = I2cAddr.create8bit(0x28);
            I2cAddr i2CAddressRangeLeft = I2cAddr.create8bit(0x2a);

            colorSensorRight = hwMap.colorSensor.get("colorR"); //Map the sensor to the hardware
            colorSensorLeft = hwMap.colorSensor.get("colorL"); //Map the sensor to the hardware
            colorSensorRight.setI2cAddress(i2CAddressColorRight);
            colorSensorLeft.setI2cAddress(i2CAddressColorLeft);
            colorSensorRight.enableLed(false);
            colorSensorLeft.enableLed(false);

            sensorGyro = hwMap.gyroSensor.get("gyro");     //Point to the gyro in the configuration file
            mrGyro = (ModernRoboticsI2cGyro) sensorGyro;         //MR GyroNew

            touchSensor = hwMap.touchSensor.get("ts");

            servoInTake = hwMap.servo.get("intake");
            servoFeeder = hwMap.servo.get("armfeeder");
            servoBallBumper = hwMap.servo.get("ballbumper");

            // Define and Initialize Motors
            motorLF = hwMap.dcMotor.get("lf");
            motorRF = hwMap.dcMotor.get("rf");
            motorLR = hwMap.dcMotor.get("lr");
            motorRR = hwMap.dcMotor.get("rr");
            motorFeeder = hwMap.dcMotor.get("feeder");
            motorShooter = hwMap.dcMotor.get("shooter");


            motorLF.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            motorRF.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
            motorLR.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            motorRR.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors


            // Set all motors to zero power
            motorLF.setPower(0);
            motorRF.setPower(0);
            motorLR.setPower(0);
            motorRR.setPower(0);
            motorShooter.setPower(0);

            // Set all drive motors to run without encoders.
            motorFeeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Set all drive motors to run without encoders.
            // May want to use RUN_USING_ENCODERS if encoders are installed.

            motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

        if (platform.equals("comp")) {
            //Define the sensors
            ods = hwMap.opticalDistanceSensor.get("ODS");  //Map the sensor to the hardware
            colorSensorRight = hwMap.colorSensor.get("colorR"); //Map the sensor to the hardware
            touchSensor = hwMap.touchSensor.get("ts");     //Map the sensor to the hardware
            sensorGyro = hwMap.gyroSensor.get("gyro");     //Point to the gyro in the configuration file
            mrGyro = (ModernRoboticsI2cGyro) sensorGyro;         //MR GyroNew
            servoInTake = hwMap.servo.get("intake");
            servoFeeder = hwMap.servo.get("armfeeder");
            servoBallBumper = hwMap.servo.get("ballbumper");


            // Define and Initialize Motors
            motorLR = hwMap.dcMotor.get("lr");
            motorRR = hwMap.dcMotor.get("rr");
            motorFeeder = hwMap.dcMotor.get("feeder");
            motorShooter = hwMap.dcMotor.get("shooter");

            motorLR.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            motorRR.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

            // Set all motors to zero power
            motorLR.setPower(0);
            motorRR.setPower(0);
            motorFeeder.setPower(0);

            //Define the sensors
            ods = hwMap.opticalDistanceSensor.get("ODS");  //Map the sensor to the hardware
            //colorSensor = hwMap.colorSensor.get("color1"); //Map the sensor to the hardware
            touchSensor = hwMap.touchSensor.get("ts");     //Map the sensor to the hardware
            sensorGyro = hwMap.gyroSensor.get("gyro");     //Point to the gyro in the configuration file
            mrGyro = (ModernRoboticsI2cGyro) sensorGyro;         //MR GyroNew

            // Define and Initialize Motors
            motorLR = hwMap.dcMotor.get("lr");
            motorRR = hwMap.dcMotor.get("rr");
            motorLR.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
            motorRR.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
            motorLR = hwMap.dcMotor.get("feeder");

            // Set all motors to zero power

            motorLR.setPower(0);
            motorRR.setPower(0);

            motorFeeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Set all motors to run without encoders.
            // May want to use RUN_USING_ENCODERS if encoders are installed.

            motorLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}