package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ColorSensorHardwareMap extends  HardwareMapUtil
{
    /* Public OpMode members. */
    public DcMotor  leftfrontDrive   = null;
    public DcMotor  rightfrontDrive  = null;
    public DcMotor  leftbackDrive   = null;
    public DcMotor  rightbackDrive  = null;
    public ColorSensor ColorSensor = null;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftfrontDrive = HardwareInitMotor("lfD", true);
        rightbackDrive = HardwareInitMotor("rbD", false);
        leftbackDrive = HardwareInitMotor("lbD", true);
        rightfrontDrive = HardwareInitMotor("rfD", false);

        // Sensors
        NormalizedColorSensor colorSensor;
    }
}
