package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@Autonomous

public class JulianBasicProgram extends LinearOpMode {
    JulianHardwareMap robot = new JulianHardwareMap();
    private ElapsedTime runtime = new ElapsedTime();

    static final double FORWARD_SPEED = 1;

    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();

        robot.Jasper.setPower(FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {

        }
    }
}