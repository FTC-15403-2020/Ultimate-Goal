package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterHardwareMap extends  HardwareMapUtil
{
    /* Public OpMode members. */
    public DcMotor  shooterMotor   = null;
    public DcMotor  wobbleGrabMotor  = null;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // Define and Initialize Motors
        shooterMotor = HardwareInitMotor("shootM", true);
        wobbleGrabMotor = HardwareInitMotor("wobbleM", true);
    }
}

