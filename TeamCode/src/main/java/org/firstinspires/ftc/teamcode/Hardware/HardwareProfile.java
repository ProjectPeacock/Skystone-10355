package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

/*
 * This is the hardware definition for the Project Peacock Robot.  This class should be
 * instantiated in the opmode to allow access to the hardware.
 *
 * Example:   private HardwareProfile robot = new HardwareProfile();
 */

public class HardwareProfile {

    /* Public OpMode members. */
    public DcMotor motorLF = null;              //Left Front drive motor
    public DcMotor motorRF = null;              //Right Front drive motor
    public DcMotor motorLR = null;              //Left Rear drive motor
    public DcMotor motorRR = null;              //Right Rear drive motor
    public DcMotor motorLift = null;            //Lift motor
    public DcMotor motorIntake1 = null;          //Intake motor
    public DcMotor motorIntake2 = null;          //Intake motor
    public DcMotor motorLinear = null;          //Linear Actuator motor - controls angle of lifting system
//    public DcMotorEx motor4Bar = null;            //motor to control 4-bar system
    public TouchSensor touchLiftForward;        //Declare the Lift Forward Touch Sensor - indicates when lift is all the way forward
    public TouchSensor touchLiftBack;           //Declare the Lift Back Touch Sensor - indicates when lift is all the way back
    public GyroSensor sensorGyro;               //Declare the GyroNew sensor
//    public ModernRoboticsI2cGyro mrGyro;        //Declare the MR GyroNew
    public CRServo servoFoundation1;
    public CRServo servoFoundation2;
    public CRServo servoGrab;
    public CRServo servoStone;
    public CRServo servoDelivery;
    public CRServo servoSwivel;
    public CRServo servo4Bar1;
    public CRServo servo4Bar2;
    public Servo servoCapstone;
    public ColorSensor colorSensorRevStone;
    public DistanceSensor sensorProximity;
    public DistanceSensor wallRangeSensor;
    public DistanceSensor leftRange;
    public DistanceSensor rightRange;
    public DistanceSensor rearRange;

    //IMU Sensor
    public BNO055IMU imu;


    /* Constructor */
    public HardwareProfile() {

    }

    /**
     * Map all the robots hardware
     * @param ahwMap Input hardwaremap
     */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        HardwareMap hwMap;
        hwMap = ahwMap;

        //Initialize IMU hardware map value. PLEASE UPDATE THIS VALUE TO MATCH YOUR CONFIGURATION
        imu = hwMap.get(BNO055IMU.class, "imu");

        //Initialize IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

/**        //Define the I2C sensors
        sensorGyro = hwMap.gyroSensor.get("gyro");     //Point to the gyro in the configuration file
        mrGyro = (ModernRoboticsI2cGyro) sensorGyro;         //MR GyroNew

        wallRangeSensor = hwMap.get(DistanceSensor.class, "wallRangeSensor");
        leftRange = hwMap.get(DistanceSensor.class, "leftRange");
        rightRange = hwMap.get(DistanceSensor.class, "rightRange");
        rearRange = hwMap.get(DistanceSensor.class, "rearRange");


        colorSensorRevStone = hwMap.get(ColorSensor.class, "sensorProximity");
        sensorProximity = hwMap.get(DistanceSensor.class, "sensorProximity");
**/
        /*
         * Initialize the touch sensors responsible for limiting the motion of the lifting system
         * colorSensorRevStone
         */
/**        touchLiftForward = hwMap.touchSensor.get("touchLiftForward");
        touchLiftBack = hwMap.touchSensor.get("touchLiftBack");
**/
        /*
         *    Define and Initialize drive Motors
         *    Set motor direction for each motor placement
         *    Configure the motors to run with encoders
         *    Set the power to the motor to be 0
         */

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

        /*
         *    Define and Initialize accessory motors
         *    Set motor direction for each motor placement (assume forward until position is finalized)
         *    Configure the motors to run with encoders as needed
         *    Set the power to the motor to be 0
         */
/**
        motorLift = hwMap.dcMotor.get("motorLift");
        motorLift.setDirection(DcMotor.Direction.FORWARD);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setPower(0);

        motorIntake1 = hwMap.dcMotor.get("motorIntake1");
        motorIntake1.setDirection(DcMotor.Direction.FORWARD);
        motorIntake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorIntake1.setPower(0);

        motorIntake2 = hwMap.dcMotor.get("motorIntake2");
        motorIntake2.setDirection(DcMotor.Direction.FORWARD);
        motorIntake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorIntake2.setPower(0);

        motorLinear = hwMap.dcMotor.get("motorLinear");
        motorLinear.setDirection(DcMotor.Direction.FORWARD);
        motorLinear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLinear.setPower(0);
**/
        /*
        motor4Bar = (DcMotorEx)hwMap.get(DcMotor.class, "motor4Bar");
        motor4Bar.setDirection(DcMotor.Direction.FORWARD);
        motor4Bar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4Bar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor4Bar.setPower(0);
        */

        /*
         * Initialize the servo motors
         */
/**        servoFoundation1 = hwMap.crservo.get("servoFoundation1");
        servoFoundation2 = hwMap.crservo.get("servoFoundation2");
        servoGrab = hwMap.crservo.get("servoGrab");
        servoStone = hwMap.crservo.get("servoStone");
        servoDelivery = hwMap.crservo.get("servoDelivery");
        servoSwivel = hwMap.crservo.get("servoSwivel");
        servo4Bar1 = hwMap.crservo.get("servo4Bar1");
        servo4Bar2 = hwMap.crservo.get("servo4Bar2");
        servoCapstone = hwMap.servo.get("servoCapstone");
 **/
    }
}