package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
public class RangeSensor
{
    private HardwareMap hardwareMap;
    I2cDevice range;
    I2cDeviceReader rangeReader;
    byte[] rangeReadings;
    RangeSensor(HardwareMap newHardwareMap)
    {
        this.hardwareMap = newHardwareMap;
        range = hardwareMap.i2cDevice.get("range");
        rangeReader = new I2cDeviceReader(range, new I2cAddr(0x28), 0x04, 2);
        byte rangeReadings[];
    }
    public int getods()
    {
        rangeReadings = rangeReader.getReadBuffer();
        return rangeReadings[1] & 0xFF;
    }
    public int getus()
    {
        rangeReadings = rangeReader.getReadBuffer();
        return rangeReadings[0] & 0xFF;
    }
}
