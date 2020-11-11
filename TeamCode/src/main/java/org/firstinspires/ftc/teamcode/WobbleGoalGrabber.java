package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Wobble Grabber", group = "wobble")
public class WobbleGoalGrabber extends LinearOpMode {
    ShooterHardwareMap robot = new ShooterHardwareMap();
    @Override
    public void runOpMode() throws InterruptedException {
        //ShooterHardwareMap robot = new ShooterHardwareMap();
        robot.init(hardwareMap);

        final int SERVO_CLOSED  = 0;
        final int SERVO_OPEN    = 60;
        int servoTargetPos = SERVO_CLOSED;
        double mPow = 0;

        waitForStart();
        while(opModeIsActive()) {
            int servoPos = (int) robot.wobbleGrabServo.getPosition();
            mPow = gamepad1.left_trigger - gamepad1.right_trigger;
            robot.wobbleGrabMotor.setPower(mPow);

            /*if(gamepad1.left_bumper) {
                if(servoPos > SERVO_OPEN / 2) {
                    hwmap.wobbleGrabServo.setPosition(SERVO_CLOSED); }
                else {hwmap.wobbleGrabServo.setPosition(SERVO_CLOSED); }*/
            if(gamepad1.a) {
                robot.wobbleGrabServo.setPosition(SERVO_CLOSED); }
            if(gamepad1.b) {
                robot.wobbleGrabServo.setPosition(SERVO_OPEN); }
        }
    }
}
