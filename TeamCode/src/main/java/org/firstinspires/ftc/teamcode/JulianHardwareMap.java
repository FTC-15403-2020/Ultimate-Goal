package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class JulianHardwareMap {

    public DcMotor Jasper      = null;
    public DcMotor Alex   = null;
    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public JulianHardwareMap(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        Jasper  = hwMap.get(DcMotor.class, "tony");
        Alex  = hwMap.get(DcMotor.class, "");

        Jasper.setDirection(DcMotor.Direction.FORWARD);
        Alex.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        Jasper.setPower(0);
        Alex.setPower(0);

        // Not running with encoders
        Jasper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Alex.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }
}
