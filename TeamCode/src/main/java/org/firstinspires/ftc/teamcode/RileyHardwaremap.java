package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RileyHardwaremap {
    /* Public OpMode members. */


    public DcMotor Tony      = null;
    public DcMotor Alex = null;



    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public RileyHardwaremap(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors

        Tony = hwMap.get(DcMotor.class, "tony");
        Alex  = hwMap.get(DcMotor.class, "alex");

        Tony.setDirection(DcMotor.Direction.FORWARD);
        Alex.setDirection(DcMotor.Direction.FORWARD);


        // Set all motors to zero power

     Tony.setPower(0);

        // Not running with encoders



        //Servos

        //Define and Initialize


    }
}

