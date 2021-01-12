package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name="RandED Tester", group="Tensorflowww")
public class RandEDTester extends LinearOpMode {

    RandomElementDetector RandED = new RandomElementDetector();
    int RandEDNum;

    public void runOpMode() throws InterruptedException {
        if(gamepad1.a) {
            RandEDNum = RandED.RandomElementNumberV1();
            telemetry.addLine("(TestOP-1)Ring Number: " + RandEDNum);
        }
        if(gamepad1.b) {
            RandEDNum = RandED.RandomElementNumberV2();
            telemetry.addLine("(TestOP-2)Ring Number: " + RandEDNum);
        }
        telemetry.update();
    }
}
