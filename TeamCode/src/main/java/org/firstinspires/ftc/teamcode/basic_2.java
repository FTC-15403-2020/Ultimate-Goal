package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous

    public class basic_2 extends LinearOpMode {
        RileyHardwaremap robot = new RileyHardwaremap();
        private ElapsedTime runtime = new ElapsedTime();

        static final double FORWARD_SPEED = 0.6;
        static final double FORWARD_SPEED_ALEX = 2.0;


        public void runOpMode() {

            robot.init(hardwareMap);

            waitForStart();

            robot.Tony.setPower(FORWARD_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 2.0))

                robot.Alex.setPower(FORWARD_SPEED_ALEX);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 2.0)){
            }
        }
    }


