package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration;

public class SophiaBot
{
    private HardwareMap hardwareMap;
    public DcMotor backl;
    public DcMotor backr;
    public DcMotor frontl;
    public DcMotor frontr;
    public DcMotor sweeper;
    private Servo lift;
    /*private Servo left;
    private Servo right;
    private Servo pusherr;
    private Servo pusherl;
    private RangeSensor range;
    private ColorSensor line;
    private ColorSensor beacon;
    private ColorSensor lineback;
    private TouchSensor botstop;
    */
    /*private int colorvalue;
    double initl = 1;
    double initr = 0;
    double startl = 0;
    double startr = 1;
    double endl = 0.59;
    double endr = 0.66;
    double startlp = 0;
    double startrp = 1;
    double endlp = 0.4;
    double endrp = 0.6;
    private boolean touch;
    private int distance;
    */
    SophiaBot(HardwareMap newHardwareMap)
    {
        this.hardwareMap = newHardwareMap;
        lift =hardwareMap.servo.get("lift");
        sweeper = hardwareMap.dcMotor.get("sweeper");
        //left = hardwareMap.servo.get("left");
        //right = hardwareMap.servo.get("right");
        //pusherr = hardwareMap.servo.get("pusherr");
        //pusherl = hardwareMap.servo.get("pusherl");
        backl = hardwareMap.dcMotor.get("backl");
        backr = hardwareMap.dcMotor.get("backr");
        //lineback = hardwareMap.colorSensor.get("lineback");
        frontl = hardwareMap.dcMotor.get("frontl");
        frontr = hardwareMap.dcMotor.get("frontr");
        //beacon = hardwareMap.colorSensor.get("beacon");
        //line = hardwareMap.colorSensor.get("line");
        //botstop = hardwareMap.touchSensor.get("botstop");
        //range = new RangeSensor(hardwareMap);
        frontr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //beacon.setI2cAddress(I2cAddr.create8bit(0x3a));
        backr.setDirection(DcMotor.Direction.REVERSE);
        frontr.setDirection(DcMotor.Direction.REVERSE);
        //line.setI2cAddress(I2cAddr.create8bit(0x3c));
        //lineback.setI2cAddress(I2cAddr.create8bit(0x1a));
        //left.setPosition(initl);
        //right.setPosition(initr);
        //pusherr.setPosition(startrp);
        //pusherl.setPosition(startlp);
        lift.setPosition(0.5);
    }

    public void drivedirect(double leftfront,double rightfront,double leftback,double rightback)
    {
        backl.setPower(leftback);
        backr.setPower(rightback);
        frontl.setPower(leftfront);
        frontr.setPower(rightfront);
    }
    public void drive(String direction, double speed)
    {
        if(direction=="forward")
        {
            backl.setPower(speed);
            backr.setPower(speed);
            frontl.setPower(speed);
            frontr.setPower(speed);
        }
        else if (direction == "backward")
        {
            backl.setPower(-1*speed);
            backr.setPower(-1*speed);
            frontl.setPower(-1*speed);
            frontr.setPower(-1*speed);
        }
        else if (direction == "left")
        {
            backl.setPower(-1*speed);
            backr.setPower(speed);
            frontl.setPower(speed);
            frontr.setPower(-1*speed);
        }
        else if (direction == "right")
        {
            backl.setPower(speed);
            backr.setPower(-1*speed);
            frontl.setPower(-1*speed);
            frontr.setPower(speed);
        }
    }
    public void turn(String direction,double speed)
    {
        if(direction=="left")
        {
            backl.setPower(-1*speed);
            backr.setPower(speed);
            frontl.setPower(-1*speed);
            frontr.setPower(speed);
        }
        else if(direction=="right")
        {
            backl.setPower(speed);
            backr.setPower(-1*speed);
            frontl.setPower(speed);
            frontr.setPower(-1*speed);
        }
    }
    public void stop()
    {
        backl.setPower(0);
        backr.setPower(0);
        frontl.setPower(0);
        frontr.setPower(0);
    }

    public void lift(String pos)
    {
        if(pos == "Up")
        {
            //lift.setPosition();
        }
        else if(pos == "Down")
        {
            lift.setPosition(0.5);
        }
    }

    public void sweeper(String dir)
    {
        if(dir == "In")
        {
            sweeper.setPower(-1);
        }
        else if(dir == "Out")
        {
            sweeper.setPower(1);
        }
        else if(dir == "Stop")
        {
            sweeper.setPower(0);
        }
    }
}
