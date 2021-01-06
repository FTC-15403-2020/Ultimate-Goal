package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeMechanismHardwareMap extends HardwareMapUtil {

    HardwareMap hwmap = null;

    public DcMotor intakeMotor = null;
    public DcMotor  shooterMotor        = null;
    public DcMotor innerpastaMotor = null;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // Define and Initialize Motors
        intakeMotor = HardwareInitMotor("Intake", false);

        shooterMotor = HardwareInitMotor("shootM", true);
        shooterMotor.setPower(0);

        innerpastaMotor = HardwareInitMotor("Pasta", false);
    }
}
