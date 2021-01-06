package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
@TeleOp(name = "BasicOdometryTest", group = "Test!")
public class BasicOdometryTest extends LinearOpMode {
    public static int encoderValue;
    DcMotor leftFrontMotor;
    DcMotor rightFrontMotor;
    DcMotor leftBackMotor;
    DcMotor rightBackMotor;
    HardwareMap hwMap           = null;
    public void init(HardwareMap hwMap) {
        hwMap = this.hwMap;
        leftFrontMotor = hardwareMap.dcMotor.get("leftFrontDrive");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFrontDrive");
        leftBackMotor = hardwareMap.dcMotor.get("leftBackDrive");
        rightBackMotor = hardwareMap.dcMotor.get("rightBackDrive");
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderValue = leftFrontMotor.getCurrentPosition();
        telemetry.addData("encoderValue", encoderValue);
    }
    public void runOpMode(){
        init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Left Encoder", leftBackMotor.getCurrentPosition());
            telemetry.addData("Right Encoder", leftFrontMotor.getCurrentPosition());
            telemetry.addData("Middle Encoder", rightFrontMotor.getCurrentPosition());
            telemetry.update();
        }
    }

}
