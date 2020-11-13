package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;


@TeleOp (name="ShooterMechanism", group="Pushbot")
//@Disabled
public class ShooterMechanism extends LinearOpMode {

    ShooterHardwareMap robot   = new ShooterHardwareMap();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    public void runOpMode() {
        double elapTime = 0;

        while (opModeIsActive()) {

            if (gamepad1.a == true) {
                robot.aimingMotor.setPower(1);
                while (gamepad1.a == true) {
                    elapTime += 1;
                }
                if (elapTime == 5) {
                    robot.aimingMotor.setPower(0);
                    break;
                }
            }
        }
    }
}
