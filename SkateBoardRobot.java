package org.firstinspires.ftc.teamcode;
import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * <h1>SteelSerpentsRobot</h1>
 * The robot class used by the SteelSerpents FTC team. This class contains all of the robot hardware,
 * drive instructions, and other details specifically relating to controlling the robot.
 */
public class SkateBoardRobot extends SteelSerpentsRobot
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
//    private Servo left;
//    private Servo right;
//    private Servo pusherr;
//    private Servo pusherl;
    private I2cDevice range;
    private I2cDeviceReader rangeReader;
    public ModernRoboticsI2cGyro gyro;   // Hardware Device Object
    private byte rangeReadings[];
    long lastTurnTime = 0;
    double lastTurnDistance = 0;

    /**
     * Sets up the SteelSerpents robot by initializing its hardware.
     * @param newHardwareMap The robot HardwareMap that is provided by our robot's OpMode.
     */
    SkateBoardRobot(HardwareMap newHardwareMap)
    {
        super(newHardwareMap);
        this.hardwareMap = newHardwareMap;
//        servo = hardwareMap.servo.get("servo");
//        left = hardwareMap.servo.get("left");
//        right = hardwareMap.servo.get("right");
//        pusherr = hardwareMap.servo.get("pusherr");
//        pusherl = hardwareMap.servo.get("pusherl");
        backLeftMotor = hardwareMap.dcMotor.get("backl");
        backRightMotor = hardwareMap.dcMotor.get("backr");
        frontLeftMotor = hardwareMap.dcMotor.get("frontl");
        frontRightMotor = hardwareMap.dcMotor.get("frontr");
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        // get a reference to a Modern Robotics GyroSensor object.
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();
//        left.setPosition(1);
//        right.setPosition(0);
//        pusherr.setPosition(0);
        I2cDevice range;
        range = hardwareMap.i2cDevice.get("range");
        rangeReader = new I2cDeviceReader(range, new I2cAddr(0x28), 0x04, 2);
        byte rangeReadings[];
    }

    /**
     * Gets the current heading relative to when the gyro was calibrated.
     * @return
     */
    public double getHeadingInDegrees()
    {
        return gyro.getHeading();
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

    /**
     *
     * @param robotVelocity Speed of the robot, from 0 to 1
     * @param directionOfMovement The robot heading in radians, from 0 to 2pi
     * @param turnSpeed Speed of the turn, from 0 to 1
     */
    public void drive(double robotVelocity, double directionOfMovement, double turnSpeed)
    {
        this.setMotorValues(robotVelocity, directionOfMovement, turnSpeed);
        this.drive();
    }

    /**
     * Turns the robot to a new heading relative to the heading set when the gyro was calibrated.
     * @param newHeadingInDegrees
     */
    public void turnToHeading(double newHeadingInDegrees)
    {
        double marginOfError = 5.0;
        double turnSpeed = 0.15;
        while (Math.abs(this.getHeadingInDegrees() - newHeadingInDegrees) > marginOfError)
        {
            int turnDirection = -1;
            double distRight = newHeadingInDegrees - this.getHeadingInDegrees();
            double distLeft = this.getHeadingInDegrees() - newHeadingInDegrees;
            if (distRight < 0)
                distRight += 360;
            if (distLeft < 0)
                distLeft += 360;

            double turnDistance = distRight;
            if (distLeft < distRight)
            {
                turnDirection = 1;
                turnDistance = distLeft;
            }
            long thisTurnTime = System.currentTimeMillis();
            double changeInTime = (double)(thisTurnTime - lastTurnTime);
            double robotTurnVelocity = (turnDistance - lastTurnDistance) / changeInTime;
            double Kp = 0.005;
            double Kd = 10;
//            double outputSpeed = Kp * turnDistance - Kd * robotTurnVelocity;
            double outputSpeed = Kp * turnDistance;
            this.drive(0, 0, clamp(outputSpeed * turnDirection));
            lastTurnTime = System.currentTimeMillis();
            lastTurnDistance = turnDistance;
        }
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