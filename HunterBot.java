package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class HunterBot
    {
    private HardwareMap hardwareMap;
    public DcMotor backl;
    public DcMotor backr;
    public DcMotor frontl;
    public DcMotor frontr;
    private Servo left;
    private Servo right;
    private Servo pusherr;
    private Servo pusherl;
    private RangeSensor range;
    private ColorSensor line;
    private ColorSensor beacon;
    private ColorSensor lineback;
    private TouchSensor botstop;
    private int colorvalue;
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
    HunterBot(HardwareMap newHardwareMap)
    {
        this.hardwareMap = newHardwareMap;
        left = hardwareMap.servo.get("left");
        right = hardwareMap.servo.get("right");
        pusherr = hardwareMap.servo.get("pusherr");
        pusherl = hardwareMap.servo.get("pusherl");
        backl = hardwareMap.dcMotor.get("backl");
        backr = hardwareMap.dcMotor.get("backr");
        lineback = hardwareMap.colorSensor.get("lineback");
        frontl = hardwareMap.dcMotor.get("frontl");
        frontr = hardwareMap.dcMotor.get("frontr");
        beacon = hardwareMap.colorSensor.get("beacon");
        line = hardwareMap.colorSensor.get("line");
        botstop = hardwareMap.touchSensor.get("botstop");
        range = new RangeSensor(hardwareMap);
        frontr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        beacon.setI2cAddress(I2cAddr.create8bit(0x3a));
        backr.setDirection(DcMotor.Direction.REVERSE);
        frontr.setDirection(DcMotor.Direction.REVERSE);
        line.setI2cAddress(I2cAddr.create8bit(0x3c));
        lineback.setI2cAddress(I2cAddr.create8bit(0x1a));
        left.setPosition(initl);
        right.setPosition(initr);
        pusherr.setPosition(startrp);
        pusherl.setPosition(startlp);
    }
    /*
    public boolean touch(String sensor)
    {
        if(sensor=="botstop")
        {
            if(botstop.isPressed())
            {
                touch = true;
            }
            else if(!botstop.isPressed())
            {
                touch = false;
            }
        }
        return touch;
    }
    */
    public void refresh()
    {
        stop();
        push(null);
        grab(null);
        beacon.enableLed(false);
        line.enableLed(false);
        lineback.enableLed(false);
        beacon.enableLed(true);
        line.enableLed(true);
        line.enableLed(true);
        beacon.enableLed(false);
        line.enableLed(false);
        lineback.enableLed(false);
    }
    public int us()
    {
        return range.getus();
    }
    public int ods()
    {
        return range.getods();
    }
    public boolean touch() {
        if (botstop.isPressed()) {
            touch = true;
        } else if (!botstop.isPressed()) {
            touch = false;
        }
        return touch;
    }
    public int color(String sensor,String color,boolean onoroff)
    {
        if(sensor=="beacon")
        {
            if(onoroff)
            {
                beacon.enableLed(true);
            }
            else if(!onoroff)
            {
                beacon.enableLed(false);
            }
            if(color=="white")
            {
                colorvalue=beacon.alpha();
            }
            else if(color=="red")
            {
                colorvalue=beacon.red();
            }
            else if(color=="blue")
            {
                colorvalue=beacon.blue();
            }
            else if(color=="green")
            {
                colorvalue=beacon.green();
            }
        }
        if(sensor=="lineback")
        {
            if(onoroff)
            {
                lineback.enableLed(true);
            }
            else if(!onoroff)
            {
                lineback.enableLed(false);
            }
            if(color=="white")
            {
                colorvalue=lineback.alpha();
            }
            else if(color=="red")
            {
                colorvalue=lineback.red();
            }
            else if(color=="blue")
            {
                colorvalue=lineback.blue();
            }
            else if(color=="green")
            {
                colorvalue=lineback.green();
            }
        }
        else if(sensor=="line")
        {
            if(onoroff)
            {
                line.enableLed(true);
            }
            else if(!onoroff)
            {
                line.enableLed(false);
            }
            if(color=="white")
            {
                colorvalue=line.alpha();
            }
            else if(color=="red")
            {
                colorvalue=line.red();
            }
            else if(color=="blue")
            {
                colorvalue=line.blue();
            }
            else if(color=="green")
            {
                colorvalue=line.green();
            }
        }
        return colorvalue;
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
    public void grab(String which)
    {
        if(which=="left")
        {
            left.setPosition(endl);
            right.setPosition(startr);
        }
        else if(which=="right")
        {
            right.setPosition(endr);
            left.setPosition(startl);
        }
        else if(which=="both")
        {
            right.setPosition(endr);
            left.setPosition(endl);
        }
        else if(which=="rest")
        {
            left.setPosition(initl);
            right.setPosition(initr);
        }
        else
        {
            left.setPosition(startl);
            right.setPosition(startr);
        }
    }
    public void push(String which)
    {
        if(which=="left")
        {
            pusherl.setPosition(endlp);
            pusherr.setPosition(startrp);
        }
        else if(which=="right")
        {
            pusherr.setPosition(endrp);
            pusherl.setPosition(startlp);
        }
        else if(which=="both")
        {
            pusherr.setPosition(endrp);
            pusherl.setPosition(endlp);
        }
        else
        {
            pusherl.setPosition(startlp);
            pusherr.setPosition(startrp);
        }
    }
}
