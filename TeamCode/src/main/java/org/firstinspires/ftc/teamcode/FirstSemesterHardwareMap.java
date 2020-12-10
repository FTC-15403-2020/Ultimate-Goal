package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FirstSemesterHardwareMap extends  HardwareMapUtil
{
    HardwareMap hwmap       = null;

    /* Public OpMode members. */
    public DcMotor  shooterMotor        = null;
    public DcMotor  wobbleGrabMotor     = null;
    public Servo    wobbleGrabServo     = null;
    public DcMotor linearActuator = null;
    public DcMotor  leftfrontDrive   = null;
    public DcMotor  rightfrontDrive  = null;
    public DcMotor  leftbackDrive   = null;
    public DcMotor  rightbackDrive  = null;


    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        shooterMotor = HardwareInitMotor("shootM", true);
        wobbleGrabMotor = HardwareInitMotor("wobbleG", true);
        wobbleGrabServo = HardwareInitServo("wobbleS", 0);
        leftfrontDrive = HardwareInitMotor("lfD", true);
        rightbackDrive = HardwareInitMotor("rbD", false);
        leftbackDrive = HardwareInitMotor("lbD", true);
        rightfrontDrive = HardwareInitMotor("rfD", false);

    }
}

