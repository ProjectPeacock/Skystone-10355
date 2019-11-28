package org.firstinspires.ftc.teamcode.Hardware;

import android.text.method.Touch;

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
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

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
    public DcMotor motorLF = null;              //Left Front drive motor
    public DcMotor motorRF = null;              //Right Front drive motor
    public DcMotor motorLR = null;              //Left Rear drive motor
    public DcMotor motorRR = null;              //Right Rear drive motor
    public DcMotor motorLift = null;            //Lift motor
    public DcMotor motorIntake = null;          //Intake motor
    public DcMotor motorLinear = null;          //Linear Actuator motor - controls angle of lifting system
    public DcMotor motor4Bar = null;            //motor to control 4-bar system
    public DcMotor motorGrab = null;            //motor for grab mechanism
    public DcMotor motorIntakeFlip = null;
    public OpticalDistanceSensor ods;           //Declare the optical distance sensor
    public TouchSensor touchLiftUp;             //Declare the Lift Up Touch Sensor - indicates when lift is all the way up
    public TouchSensor touchLiftDown;           //Declare the Lift Down Touch Sensor - indicates when lift is all the way down
    public TouchSensor touchLiftForward;        //Declare the Lift Forward Touch Sensor - indicates when lift is all the way forward
    public TouchSensor touchLiftBack;           //Declare the Lift Back Touch Sensor - indicates when lift is all the way back
    public GyroSensor sensorGyro;               //Declare the GyroNew sensor
    public ModernRoboticsI2cGyro mrGyro;        //Declare the MR GyroNew
    public ModernRoboticsI2cRangeSensor rangeSensorLeft;               //Declare the left range sensor
    public ModernRoboticsI2cRangeSensor rangeSensorRight;              //Declare the right range sensor
    public ModernRoboticsI2cRangeSensor rangeSensorFront;              //Declare the front range sensor
    public ModernRoboticsI2cRangeSensor rangeSensorRear;               //Declare the rear range sensor
    public Servo servoLeftGrab;                 //Declare the left grabbing servo
    public Servo servoRightGrab;                //Declare the right grabbing servo
    public Servo servoClawClose;                //Declare the claw opening/closing servo
    public Servo servoClawRotate;
    public WebcamName webcamName = null;

    /* I2C Range Sensor members*/
    /**    I2cDevice rangeLeft;
     I2cDevice rangeRight;
     I2cDevice rangeFront;
     I2cDevice rangeBack;
     I2cDeviceSynch rangeLeftReader;
     I2cDeviceSynch rangeRightReader;
     I2cDeviceSynch rangeFrontReader;
     I2cDeviceSynch rangeBackReader;
     **/
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

        /*
         * Retrieve the camera we are to use.
         */
//        webcamName = hwMap.get(WebcamName.class, "Webcam 1");

        //Define the I2C sensors
//        ods = hwMap.opticalDistanceSensor.get("ODS");  //Map the sensor to the hardware

        I2cAddr i2CAddressRangeLeft = I2cAddr.create8bit(0x28);
        rangeSensorLeft = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensorLeft");
        rangeSensorLeft.setI2cAddress(i2CAddressRangeLeft);
//        I2cAddr i2CAddressColorRight = I2cAddr.create8bit(0x3c);
//        I2cAddr i2CAddressColorLeft = I2cAddr.create8bit(0x4c);
//        I2cAddr i2CAddressRangeLeft = I2cAddr.create8bit(0x28);
        //       rangeSensorLeft = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensorLeft");
//        rangeSensorLeft.setI2cAddress(i2CAddressRangeLeft);
//
//        I2cAddr i2CAddressRangeRight = I2cAddr.create8bit(0x28)
//        rangeSensorRight = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensorRight");
//        rangeSensorRight.setI2cAddress(i2CAddressRangeRight);

//        I2cAddr i2CAddressRangeFront = I2cAddr.create8bit(0x28);
//        rangeSensorFront = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensorFront");
//        rangeSensorFront.setI2cAddress(i2CAddressRangeFront);

//        I2cAddr i2CAddressRangeRear = I2cAddr.create8bit(0x28);
//        rangeSensorRear = hwMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensorRear");
//        rangeSensorRear.setI2cAddress(i2CAddressRangeRear);


        sensorGyro = hwMap.gyroSensor.get("gyro");     //Point to the gyro in the configuration file
        mrGyro = (ModernRoboticsI2cGyro) sensorGyro;         //MR GyroNew
//        colorSensorRight = hwMap.colorSensor.get("colorR"); //Map the sensor to the hardware
//        colorSensorLeft = hwMap.colorSensor.get("colorL"); //Map the sensor to the hardware
//        colorSensorRight.setI2cAddress(i2CAddressColorRight);
//        colorSensorLeft.setI2cAddress(i2CAddressColorLeft);
//        colorSensorRight.enableLed(false);
//        colorSensorLeft.enableLed(false);

//        sensorGyro = hwMap.gyroSensor.get("gyro");     //Point to the gyro in the configuration file
//        mrGyro = (ModernRoboticsI2cGyro) sensorGyro;         //MR GyroNew

        /**
         * Initialize the touch sensors responsible for limiting the motion of the lifting system
         **/



//        touchLiftUp = hwMap.touchSensor.get("touchLiftUp");
//        touchLiftDown = hwMap.touchSensor.get("touchLiftDown");
//        touchLiftForward = hwMap.touchSensor.get("touchLiftForward");
//        touchLiftBack = hwMap.touchSensor.get("touchLiftBack");
//        touchLiftUp = hwMap.touchSensor.get("touchLiftUp");
//        touchLiftDown = hwMap.touchSensor.get("touchLiftDown");
        touchLiftForward = hwMap.touchSensor.get("touchLiftForward");
        touchLiftBack = hwMap.touchSensor.get("touchLiftBack");



         //touchLiftUp = hwMap.touchSensor.get("touchLiftUp");
         //touchLiftDown = hwMap.touchSensor.get("touchLiftDown");
         touchLiftForward = hwMap.touchSensor.get("touchLiftForward");
         touchLiftBack = hwMap.touchSensor.get("touchLiftBack");
         //        touchLiftUp = hwMap.touchSensor.get("touchLiftUp");
         //        touchLiftDown = hwMap.touchSensor.get("touchLiftDown");
         //        touchLiftForward = hwMap.touchSensor.get("touchLiftForward");
        //       touchLiftBack = hwMap.touchSensor.get("touchLiftBack");



        /**
         *    Define and Initialize drive Motors
         *    Set motor direction for each motor placement
         *    Configure the motors to run with encoders
         *    Set the power to the motor to be 0
         **/

        motorLF = hwMap.dcMotor.get("motorLF");
        motorLF.setDirection(DcMotor.Direction.FORWARD);
        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLF.setPower(0);

        motorRF = hwMap.dcMotor.get("motorRF");
        motorRF.setDirection(DcMotor.Direction.REVERSE);        // motors placed at the rear of the robot need to be set to run in reverse
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRF.setPower(0);

        motorLR = hwMap.dcMotor.get("motorLR");
        motorLR.setDirection(DcMotor.Direction.FORWARD);
        motorLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLR.setPower(0);

        motorRR = hwMap.dcMotor.get("motorRR");
        motorRR.setDirection(DcMotor.Direction.REVERSE);        // motors placed at the rear of the robot need to be set to run in reverse
        motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRR.setPower(0);

        /**
         *    Define and Initialize accessory motors
         *    Set motor direction for each motor placement (assume forward until position is finalized)
         *    Configure the motors to run with encoders as needed
         *    Set the power to the motor to be 0
         **/

        motorLift = hwMap.dcMotor.get("motorLift");
        motorLift.setDirection(DcMotor.Direction.FORWARD);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setPower(0);

        motorIntake = hwMap.dcMotor.get("motorIntake");
        motorIntake.setDirection(DcMotor.Direction.FORWARD);
        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorIntake.setPower(0);

        motorLinear = hwMap.dcMotor.get("motorLinear");
        motorLinear.setDirection(DcMotor.Direction.FORWARD);
        motorLinear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLinear.setPower(0);

        motor4Bar = hwMap.dcMotor.get("motor4Bar");
        motor4Bar.setDirection(DcMotor.Direction.FORWARD);
        motor4Bar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4Bar.setPower(0);

        /**
         * Initialize the servo motors
         */


        servoLeftGrab = hwMap.servo.get("servoLeftGrab");
        servoRightGrab = hwMap.servo.get("servoRightGrab");
        servoClawClose = hwMap.servo.get("servoClawClose");
        servoClawRotate = hwMap.servo.get("servoClawRotate");

        servoLeftGrab = hwMap.servo.get("servoLeftGrab");
        servoRightGrab = hwMap.servo.get("servoRightGrab");
        servoClawClose = hwMap.servo.get("servoClawClose");
        servoClawRotate = hwMap.servo.get("servoClawRotate");
    }
}