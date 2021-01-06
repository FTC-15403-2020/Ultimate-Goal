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
        double elapTime = System.currentTimeMillis();
        robot.init(hardwareMap);
        double powershoot = 0;
        double timeShooting = 0;
        double CurTime = elapTime;
        double LastTime = elapTime;
        double poweraiming = 0;
        double aimTime = 0;

        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.right_bumper==true) {
                double period = 5000;
                double quadScale = 0.1;
                CurTime = elapTime;
                timeShooting = timeShooting + (LastTime-CurTime);
                LastTime = CurTime;

                if(timeShooting > period) {
                    //power = -1 * (timeShooting / period);
                    //power = -quadScale * ((timeShooting/period) * (timeShooting/period));
                    powershoot = Math.pow((timeShooting/period), 3);
                }
                else {
                    powershoot = -1;
                }
            }
            else {
                timeShooting = 0;
                powershoot = 0;
            }

            // Send calculated power to wheels
            robot.shooterMotor.setPower(powershoot);

            if (gamepad1.a == true) {
                poweraiming = 0.5;
            }
            while (gamepad1.a == true) {
                aimTime += 1;

                if (aimTime == 10) {
                    poweraiming = 0;
                    break;
                }
                //robot.aimingMotor.setPower(poweraiming);
            }
        }
    }
}
