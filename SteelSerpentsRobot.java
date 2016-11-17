package org.firstinspires.ftc.teamcode;
import android.util.Log;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * <h1>SteelSerpentsRobot</h1>
 * The robot class used by the SteelSerpents FTC team. This class contains all of the robot hardware,
 * drive instructions, and other details specifically relating to controlling the robot.
 */
public class SteelSerpentsRobot
{

    public float DEAD_ZONE = 0.15f;
    private HardwareMap hardwareMap;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private double backLeftPower   = 0;
    private double backRightPower  = 0;
    private double frontLeftPower  = 0;
    private double frontRightPower = 0;
//    private Servo servo;
    private Servo left;
    private Servo right;
    private Servo pusherr;
    private Servo pusherl;
    private I2cDeviceReader rangeReader;
    private byte rangeReadings[];
    public DcMotor sweeper;
    private Servo lift;

    private ColorSensor lineback;
    private RangeSensor range;
    private ColorSensor line;
    private ColorSensor beacon;
    private TouchSensor botstop;
    double startlp = 0;
    double startrp = 1;
    double initl = 1;
    double initr = 0;

    /**
     * Sets up the SteelSerpents robot by initializing its hardware.
     * @param newHardwareMap The robot HardwareMap that is provided by our robot's OpMode.
     */
    SteelSerpentsRobot(HardwareMap newHardwareMap)
    {
        this.hardwareMap = newHardwareMap;
        lift =hardwareMap.servo.get("lift");
        sweeper = hardwareMap.dcMotor.get("sweeper");
        //left = hardwareMap.servo.get("left");
        //right = hardwareMap.servo.get("right");
        //pusherr = hardwareMap.servo.get("pusherr");
        //pusherl = hardwareMap.servo.get("pusherl");
        //lineback = hardwareMap.colorSensor.get("lineback");
        frontLeftMotor = hardwareMap.dcMotor.get("frontl");
        frontRightMotor = hardwareMap.dcMotor.get("frontr");
        backLeftMotor = hardwareMap.dcMotor.get("backl");
        backRightMotor = hardwareMap.dcMotor.get("backr");
        //beacon = hardwareMap.colorSensor.get("beacon");
        //line = hardwareMap.colorSensor.get("line");
        //botstop = hardwareMap.touchSensor.get("botstop");
        //range = new RangeSensor(hardwareMap);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //beacon.setI2cAddress(I2cAddr.create8bit(0x3a));
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        //line.setI2cAddress(I2cAddr.create8bit(0x3c));
        //lineback.setI2cAddress(I2cAddr.create8bit(0x1a));
        //left.setPosition(initl);
        //right.setPosition(initr);
        //pusherr.setPosition(startrp);
        //pusherl.setPosition(startlp);
        lift.setPosition(0.5);
    }
    /**
     * Gets the speed for the front left wheel.
     * Based on this paper <a href="http://thinktank.wpi.edu/resources/346/ControllingMecanumDrive.pdf">Controlling Mecanum Drive</a>
     */
    private double getFrontLeftMecanumVelocity(double robotVelocity, double robotHeading, double robotAngularVelocity)
    {
        return robotVelocity * Math.sin(robotHeading + (Math.PI / 4)) + robotAngularVelocity;
    }
    /**
     * Gets the speed for the front right wheel.
     * Based on this paper <a href="http://thinktank.wpi.edu/resources/346/ControllingMecanumDrive.pdf">Controlling Mecanum Drive</a>
     */
    private double getFrontRightMecanumVelocity(double robotVelocity, double robotHeading, double robotAngularVelocity)
    {
        return robotVelocity * Math.cos(robotHeading + (Math.PI / 4)) - robotAngularVelocity;
    }
    /**
     * Gets the speed for the back left wheel.
     * Based on this paper <a href="http://thinktank.wpi.edu/resources/346/ControllingMecanumDrive.pdf">Controlling Mecanum Drive</a>
     */
    private double getBackLeftMecanumVelocity(double robotVelocity, double robotHeading, double robotAngularVelocity)
    {
        return robotVelocity * Math.cos(robotHeading + (Math.PI / 4)) + robotAngularVelocity;
    }
    /**
     * Gets the speed for the back rightwheel.
     * Based on this paper <a href="http://thinktank.wpi.edu/resources/346/ControllingMecanumDrive.pdf">Controlling Mecanum Drive</a>
     */
    private double getBackRightMecanumVelocity(double robotVelocity, double robotHeading, double robotAngularVelocity)
    {
        return robotVelocity * Math.sin(robotHeading + (Math.PI / 4)) - robotAngularVelocity;
    }

    /**
     * Sets the motor speeds to the appropriate values for a mecanum-wheeled robot. Must use the drive() command to send the drive instruction to the robot.
     * @param robotVelocity The speed at which you want the robot to translate over the gamefield. Range: [-1,1]
     * @param directionOfMovement The heading in radians in which you want the robot to translate. Range: [0,2PI)
     * @param turnSpeed The rate of rotation and direction of rotation you wish to move in. Range: [-1,1]
     */
    public void setMotorValues(double robotVelocity, double directionOfMovement, double turnSpeed)
    {
        frontLeftPower  = getFrontLeftMecanumVelocity(  robotVelocity, -directionOfMovement, -turnSpeed);
        frontRightPower = getFrontRightMecanumVelocity( robotVelocity, -directionOfMovement, -turnSpeed);
        backLeftPower   = getBackLeftMecanumVelocity(   robotVelocity, -directionOfMovement, -turnSpeed);
        backRightPower  = getBackRightMecanumVelocity(  robotVelocity, -directionOfMovement, -turnSpeed);
        print("Front Left  Power: ", frontLeftPower);
        print("Front Right Power: ", frontRightPower);
        print("Back  Left  Power: ", backLeftPower);
        print("Back  Right Power: ", backRightPower);
    }

    /**
     * Sends the actual drive command to the robot.
     */
    public void drive()
    {
        frontLeftPower  = clamp(frontLeftPower  );
        frontRightPower = clamp(frontRightPower );
        backLeftPower   = clamp(backLeftPower   );
        backRightPower  = clamp(backRightPower  );
        frontLeftMotor.setPower(    frontLeftPower  );
        frontRightMotor.setPower(   frontRightPower );
        backLeftMotor.setPower(     backLeftPower   );
        backRightMotor.setPower(    backRightPower  );
    }


    public double clamp(double value, int min, int max)
    {
        if (value < min)
        {
            print("Clamp", value);
            value = min;
        }
        else if (max < value)
        {
            print("Clamp", value);
            value = max;
        }
        return value;
    }
    public double clamp (double value)
    {
        return clamp(value, -1, 1);
    }
    public void print(String key, String text)
    {
        Log.d(key, text);
    }
    public void print(String key, double text)
    {
        print(key, String.valueOf(text));
    }
}