package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Wobble Grabber", group = "wobble")
public class WobbleGoalGrabber extends LinearOpMode {
    HardwareMap hwmap = null;
    @Override
    public void runOpMode() throws InterruptedException {
        ShooterHardwareMap robot = new ShooterHardwareMap();
        robot.init(hardwareMap);

        waitForStart();
        while(opModeIsActive()) {
            double powa = gamepad1.left_trigger - gamepad1.right_trigger;
            robot.wobbleGrabMotor.setPower(powa);
        }
    }
}
