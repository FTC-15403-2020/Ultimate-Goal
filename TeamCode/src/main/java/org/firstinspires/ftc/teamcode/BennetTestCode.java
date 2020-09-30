package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Bennet Test Code", group="Mecanum_Drive")
public class BennetTestCode extends LinearOpMode {

    //Declare Drive Base motors
    DcMotor lfMotor;
    DcMotor rfMotor;
    DcMotor lbMotor;
    DcMotor rbMotor;

    //Declare Servos
    Servo servo1;

    //Variables
    static final double     FORWARD_SPEED = 0.6;

    public void runOpMode() throws InterruptedException {

        //Grab Drive Base motors from Hardware Map
        lfMotor = hardwareMap.dcMotor.get("leftFrontDrive");
        rfMotor = hardwareMap.dcMotor.get("rightFrontDrive");
        lbMotor = hardwareMap.dcMotor.get("leftBackDrive");
        rbMotor = hardwareMap.dcMotor.get("rightBackDrive");

        //Grab Drive Base motors from Hardware Map
        servo1 = hardwareMap.servo.get("servo1");

        //RUN LOOP-------------------------------------------
        while(opModeIsActive()) {
            while(getRuntime() < time) {
                lfMotor.setPower(FORWARD_SPEED);
                rfMotor.setPower(FORWARD_SPEED);
                lbMotor.setPower(FORWARD_SPEED);
                rbMotor.setPower(FORWARD_SPEED);
            }
            if(gamepad1.left_trigger > 0) {
                servo1.setPosition(310);
                telemetry.addLine("setting servo to 310");
            }
            if(gamepad1.right_trigger > 0) {
                servo1.setPosition(50);
                telemetry.addLine("setting servo to 50");
            }
        }

    }
}
