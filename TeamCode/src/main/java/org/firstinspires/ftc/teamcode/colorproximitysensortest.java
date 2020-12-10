package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp (name="colorproximitysensortest", group="hello")
//@Disabled
public class colorproximitysensortest extends LinearOpMode {

    /* Declare OpMode members. */
    ColorSensorHardwareMap robot = new ColorSensorHardwareMap() ;
    private ElapsedTime runtime = new ElapsedTime();

    /*
    static final double FORWARD_SPEED = 0.6;
    static double turnPower = 0.5;
    static double strafePower = 0.5;
    static double fwdBackPower = 0.5;
    static double leftFrontPower;
    static double leftBackPower;
    static double rightFrontPower;
    static double rightBackPower;
    static double color_sensor;
     */

    protected void runSample() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
    }
    NormalizedColorSensor colorSensor;

    @Override
    public void runOpMode() {


        robot.init(hardwareMap);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();




        while (opModeIsActive()) {
            /*
            fwdBackPower = gamepad1.left_stick_y;
            strafePower = gamepad1.left_stick_x;
            turnPower = -gamepad1.right_stick_x;
            leftFrontPower = fwdBackPower - turnPower - strafePower;
            rightFrontPower = fwdBackPower + turnPower + strafePower;
            leftBackPower = fwdBackPower - turnPower + strafePower;
            rightBackPower = fwdBackPower + turnPower - strafePower;
            robot.leftfrontDrive.setPower(-leftFrontPower);
            robot.rightfrontDrive.setPower(-rightFrontPower);
            robot.leftbackDrive.setPower(-leftBackPower);
            robot.rightbackDrive.setPower(-rightBackPower);
            telemetry.addData("Speed", robot.rightbackDrive.getPower());
            telemetry.update();
            */
            //robot.color_sensor.red();
            //robot.color_sensor.green();
            //robot.color_sensor.blue();

            //robot.color_sensor.alpha();
            //robot.color_sensor.argb();

            runSample();
        }
    }
}
