package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name="IntakeMechanism", group="Pushbot")
public class IntakeMechanism extends LinearOpMode{
   IntakeMechanismHardwareMap robot = new    IntakeMechanismHardwareMap() ;

   static double intakeSpeed;

   public void runOpmode() {

       robot.init(hardwareMap);

       waitForStart();

       while (opModeIsActive()) {

           intakeSpeed = gamepad1.right_stick_y;
       }

    }
}
