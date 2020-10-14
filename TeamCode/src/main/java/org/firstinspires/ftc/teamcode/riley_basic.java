package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;


@Autonomous

public class riley_basic extends LinearOpMode {
    RileyHardwaremap robot = new RileyHardwaremap();
    private ElapsedTime runtime = new ElapsedTime();

    static final double FORWARD_SPEED = 0.6;


    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();

        robot.Tony.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
        }
    }
}

