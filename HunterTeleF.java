package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name = "Drive", group = "TeleOp")
public class HunterTeleF extends OpMode
{
    HunterBot robot;
    boolean tog2 = false;
    double speedf = 3;
    double runspeed;
    double backl;
    double backr;
    double frontl;
    double frontr;
    boolean rest = true;
    boolean tog1 = false;
    @Override
    public void init()
    {
        robot = new HunterBot(hardwareMap);
        robot.refresh();
    }
    @Override
    public void loop()
    {
        //Reads
        telemetry.addData("Red",robot.color("line","red",true));
        telemetry.addData("Blue",robot.color("line","blue",true));
        telemetry.addData("White",robot.color("line","white",true));
        telemetry.addData("Green",robot.color("line","green",true));
        telemetry.addData("Red",robot.color("beacon","red",false));
        telemetry.addData("Blue",robot.color("beacon","blue",false));
        telemetry.addData("White",robot.color("beacon","white",false));
        telemetry.addData("Green",robot.color("beacon","green",false));
        telemetry.addData("US",robot.us());
        telemetry.addData("ODS",robot.ods());
        //telemetry.addData("BL: ", robot.backl.getCurrentPosition());
        //telemetry.addData("FL: ", robot.frontl.getCurrentPosition());
        //telemetry.addData("BR: ", robot.backr.getCurrentPosition());
        //telemetry.addData("FR: ", robot.frontr.getCurrentPosition());
        //Drive Speed
        if(gamepad1.start&&!tog2)
        {
            if(speedf==3)
            {
                speedf = 1;
            }
            else
            {
                speedf++;
            }
            tog2 = true;
        }
        else if(!gamepad1.start&&tog2)
        {
            tog2 = false;
        }
        if(speedf == 1)
        {
            runspeed = 1;
        }
        else if(speedf == 2)
        {
            runspeed = 0.6;
        }
        else if(speedf == 3)
        {
            runspeed = 0.3;
        }
        //GamePad Drive
        if(gamepad1.dpad_right)
        {
            backl=-1*runspeed;
            backr=runspeed;
            frontl=runspeed;
            frontr=-1*runspeed;
        }
        else if(gamepad1.dpad_left)
        {
            backl=runspeed;
            backr=-1*runspeed;
            frontl=-1*runspeed;
            frontr=runspeed;
        }
        else if(gamepad1.dpad_up)
        {
            backl=runspeed;
            backr=runspeed;
            frontl=runspeed;
            frontr=runspeed;
        }
        else if(gamepad1.dpad_down)
        {
            backl=-1*runspeed;
            backr=-1*runspeed;
            frontl=-1*runspeed;
            frontr=-1*runspeed;
        }
        else
        {
            //Left Drive
            if (gamepad1.left_bumper)
            {
                backl=runspeed;
                frontl=runspeed;
            }
            else if (gamepad1.left_trigger > 0.5f)
            {
                backl=-1*runspeed;
                frontl=-1*runspeed;
            }
            else
            {
                if(Math.abs(gamepad1.left_stick_y)>0.15)
                {
                    backl=(-1*gamepad1.left_stick_y)/speedf;
                    frontl=(-1*gamepad1.left_stick_y) / speedf;
                }
                else
                {
                    backl=0;
                    frontl=0;
                }
            }
            //Right Drive
            if (gamepad1.right_bumper)
            {
                backr=runspeed;
                frontr=runspeed;
            }
            else if (gamepad1.right_trigger > 0.5f)
            {
                backr=-1*runspeed;
                frontr=-1*runspeed;
            }
            else
            {
                if(Math.abs(gamepad1.right_stick_y)>0.15)
                {
                    backr=(-1*gamepad1.right_stick_y)/speedf;
                    frontr=(-1*gamepad1.right_stick_y)/speedf;
                }
                else
                {
                    backr=0;
                    frontr=0;
                }
            }
        }
        robot.drivedirect(frontl,frontr,backl,backr);
        //Pusher
        if(gamepad1.a&&!gamepad1.y)
        {
            robot.push("left");
        }
        else if(gamepad1.y&&!gamepad1.a)
        {
            robot.push("right");
        }
        else if(gamepad1.a&&gamepad1.y)
        {
            robot.push("both");
        }
        else
        {
            robot.push("none");
        }
        //Grabber
        if(rest)
        {
            robot.grab("rest");
            if(!tog1&&gamepad1.back)
            {
                tog1 = true;
                rest = false;
            }
            else if(tog1&&!gamepad1.back)
            {
                tog1 = false;
            }
        }
        else if(!rest)
        {
            if(!tog1&&gamepad1.back)
            {
                tog1 = true;
                rest = true;
            }
            else if(tog1&&!gamepad1.back)
            {
                tog1 = false;
            }
            if (!gamepad1.b && gamepad1.x)
            {
                robot.grab("left");
            }
            else if (!gamepad1.x && gamepad1.b)
            {
                robot.grab("right");
            }
            else if (gamepad1.x && gamepad1.b)
            {
                robot.grab("both");
            }
            else
            {
                robot.grab("none");
            }
        }
    }
    @Override
    public void stop()
    {

    }
}
