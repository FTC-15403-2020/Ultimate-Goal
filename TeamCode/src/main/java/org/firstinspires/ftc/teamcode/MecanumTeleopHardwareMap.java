package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MecanumTeleopHardwareMap extends HardwareMapUtil{
    HardwareMap hwMap =null;
    public DcMotor leftfrontDrive = null;
    public DcMotor leftbackDrive = null;
    public DcMotor rightfrontDrive = null;
    public DcMotor rightbackDrive = null;
    public void init(HardwareMap ahwMap){
        leftfrontDrive = HardwareInitMotor ("LF", true);
        leftbackDrive = HardwareInitMotor ("LB" , true);
        rightfrontDrive = HardwareInitMotor ("RF" , true);
        rightbackDrive = HardwareInitMotor ("RB" , true);
    }


    //leftfrontDrive = hwMap.get(DcMotor.class, "LF")
    //leftbackDrive = hwMap.get(DcMotor.class, "LB")
    //rightfrontDrive = hwMap.get(DcMotor.class, "RF")
    //rightbackDrive = hwMap.get(DcMotor.class, "RB")
}
