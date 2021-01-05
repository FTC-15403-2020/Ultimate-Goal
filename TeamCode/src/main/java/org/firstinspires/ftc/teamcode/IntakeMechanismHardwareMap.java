package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeMechanismHardwareMap extends HardwareMapUtil {

    HardwareMap hwmap = null;

    public DcMotor intakeMotor = null;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // Define and Initialize Motors
        intakeMotor = HardwareInitMotor("Intake", false);
    }
}