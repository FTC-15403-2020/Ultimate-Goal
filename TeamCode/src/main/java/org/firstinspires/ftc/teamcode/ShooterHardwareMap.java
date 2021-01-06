package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterHardwareMap extends  HardwareMapUtil
{
    HardwareMap hwmap       = null;

    /* Public OpMode members. */
    public DcMotor  shooterMotor        = null;
   // public DcMotor  wobbleGrabMotor     = null;
    //public Servo    wobbleGrabServo     = null;
    //public DcMotor linearActuator = null;
   // public DcMotor aimingMotor = null;
    public DcMotor intakeMotor = null;


    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // Define and Initialize Motors
        /*shooterMotor = HardwareInitMotor("shootM", true);
        wobbleGrabMotor = HardwareInitMotor("wobbleM", false);
        wobbleGrabServo = HardwareInitServo("wobbleS", 0);*/

        shooterMotor    = hwmap.get(DcMotor.class, "shootM");
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setPower(0);

        //wobbleGrabMotor = hwmap.get(DcMotor.class, "wobbleG");
        //wobbleGrabServo = hwmap.get(Servo.class, "wobbleS");
        //wobbleGrabServo.setPosition(0);
        //linearActuator = HardwareInitMotor ("lfD", true);

        /*aimingMotor = hwmap.get(DcMotor.class, "Aiming");
        aimingMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        aimingMotor.setPower(0); */
        intakeMotor = HardwareInitMotor("Intake", false);



        //shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}

