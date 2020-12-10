package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Wobble Grabber", group = "wobble")
public class WobbleGoalGrabber extends LinearOpMode {
    DcMotor rotationMotor;
    Servo grabberServo;

    public void runOpMode() throws InterruptedException {
        rotationMotor = hardwareMap.dcMotor.get("wobbleG");
        grabberServo = hardwareMap.servo.get("wobbleS");
        grabberServo.setDirection(Servo.Direction.REVERSE);

        final int SERVO_CLOSED  = 359;
        final int SERVO_OPEN    = 60;

        double mPow = 0;

        waitForStart();
        while(opModeIsActive()) {
            //int servoPos = (int) grabberServo.getPosition();
            //mPow = gamepad1.left_trigger - gamepad1.right_trigger;
            mPow = gamepad1.left_stick_y/2;
            rotationMotor.setPower(mPow);

            /*if(gamepad1.left_bumper) {
                if(servoPos > SERVO_OPEN / 2) {
                    hwmap.wobbleGrabServo.setPosition(SERVO_CLOSED); }
                else {hwmap.wobbleGrabServo.setPosition(SERVO_CLOSED); }*/
            if(gamepad1.a) {
                grabberServo.setPosition(SERVO_CLOSED); }
            if(gamepad1.b) {
                grabberServo.setPosition(SERVO_OPEN); }
        }
    }
}
