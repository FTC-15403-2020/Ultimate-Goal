package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp

    public class basic_2 extends LinearOpMode {
        ShooterHardwareMap robot = new ShooterHardwareMap();
        private ElapsedTime runtime = new ElapsedTime();

        static final double FORWARD_SPEED = 0.6;
        static final double FORWARD_SPEED_ALEX = 1.0;
        static double turnPower = 0.5;
        static double strafePower = 0.5;
        static double fwdBackPower = 0.5;
        static double leftFrontPower;
        static double leftBackPower;
        static double rightFrontPower;
        static double rightBackPower;



    public void runOpMode() {

            robot.init(hardwareMap);

            waitForStart();


        while (opModeIsActive()){
            fwdBackPower = gamepad1.left_stick_y;
            strafePower = gamepad1.left_stick_x;
            turnPower = gamepad1.right_stick_y;
            leftFrontPower = fwdBackPower - turnPower - strafePower;
            rightFrontPower = fwdBackPower + turnPower + strafePower;
            leftBackPower = fwdBackPower - turnPower + strafePower;
            rightBackPower = fwdBackPower + turnPower - strafePower;
            /*robot.leftfrontDrive.setPower(leftFrontPower);
            robot.rightfrontDrive.setPower(rightFrontPower);
            robot.leftbackDrive.setPower(leftBackPower);
            robot.rightbackDrive.setPower(rightBackPower);*/
        }

    }
}

