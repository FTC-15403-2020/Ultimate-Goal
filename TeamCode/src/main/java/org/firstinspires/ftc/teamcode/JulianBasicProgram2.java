package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@Autonomous

public class JulianBasicProgram2 extends LinearOpMode {
    JulianHardwareMap robot = new JulianHardwareMap();
    private ElapsedTime runtime = new ElapsedTime();

    static final double FORWARD_SPEED_JASPER = 1;
    static final double FORWARD_SPEED_ALEX = -2;

    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();

        robot.Jasper.setPower(FORWARD_SPEED_JASPER);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {

        }
        robot.Alex.setPower(FORWARD_SPEED_ALEX);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
        }
    }
}
