package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name = "Red", group = "Autonomous")
public class HunterAutoR extends LinearOpMode
{
    HunterBot robot;
    double speedf = 3;
    double runspeed;
    double backl;
    double backr;
    double frontl;
    double frontr;
    String teamcolor = "red";
    String notteamcolor = "blue";
    int valueofred = 3;
    int valueofblue = 3;
    double updatetime;
    double maxtime;
    boolean settime = false;
    boolean timereach = false;
    double speed = 0.12;
    boolean linefind = false;
    int colort = 15; // Threshold for White Color
    boolean reset = false;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HunterBot(hardwareMap);
        waitForStart();
        robot.refresh();
        //Beginning Of Code
        while (robot.color("line", "white",true) < colort)
        {
            robot.drive("forward", speed); //Drive forward until line sensor identifies white color on line
        }
        robot.stop();

        while(!robot.touch()&&!linefind)
        {

            if(robot.color("line", "white",true)>colort)
            {
                robot.drive("forward", speed);
                if(!reset)
                {
                    timereset();
                    reset = true;
                }
                if(!timecheck(2))
                {
                    linefind = true;
                }
            }
            else if(robot.color("line", "white",true)<colort)
            {
                if(reset)
                {
                    timereset();
                    reset = false;
                }
                if(timecheck(2))
                {
                    robot.drivedirect(-1*speed,0,-1*speed,0);
                }
                else
                {
                    robot.drivedirect(speed, 0, speed, 0);
                }
            }
        }
        robot.stop();
        colorchooser();
    }
    public boolean timecheck(double seconds)
    {
        if(!settime)
        {
            maxtime = time+seconds;
            settime = true;
        }
        updatetime = time;
        if(updatetime<maxtime)
        {
            timereach = true;
        }
        else if(updatetime>=maxtime)
        {
            timereach = false;
        }
        return timereach;
    }
    public void timereset()
    {
        settime = false;
    }
    public void colorchooser()
    {
        robot.grab("both");
        try
        {
            Thread.sleep(2000);
        }
        catch (InterruptedException ex)
        {
            Thread.currentThread().interrupt();
        }
        if(robot.color("beacon", teamcolor,false) >= valueofred)
        {
            robot.push("left");
            try
            {
                Thread.sleep(2000);
            }
            catch (InterruptedException ex)
            {
                Thread.currentThread().interrupt();
            }
            robot.push("none");
            robot.grab("none");
        }
        else if(robot.color("beacon", notteamcolor,false) >= valueofblue)
        {
            robot.push("right");
            try
            {
                Thread.sleep(2000);
            }
            catch (InterruptedException ex)
            {
                Thread.currentThread().interrupt();
            }
            robot.push("none");
            robot.grab("none");
        }
    }
}
