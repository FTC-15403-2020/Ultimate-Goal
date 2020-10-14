
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

import java.util.Locale;

@TeleOp(name="IMU_Test", group="Mecanum_Drive")
@Disabled
public class AutoUtil extends LinearOpMode {
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159);
    static final double DRIVE_SPEED = 0.2;
    static final double STRAFE_SPEED = 0.8;
    private ElapsedTime     runtime = new ElapsedTime();

    //Motors
    DcMotor leftFrontMotor;
    DcMotor rightFrontMotor;
    DcMotor leftBackMotor;
    DcMotor rightBackMotor;
    DcMotor armRotationMotor;
    DcMotor armExtensionMotor;
    //Servos
    Servo leftServo;
    Servo rightServo;
    Servo stoneServo;
    Servo grabberServo;


    // The IMU sensor object
    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    private ElapsedTime period = new ElapsedTime();

    public void runOpMode() throws InterruptedException {

        sleep(100);
        waitForStart();





        // Start the logging of measured acceleration
        //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        /*while (opModeIsActive()) {
            telemetry.addData("Heading", heading(imu));
            telemetry.update();
        }
        */

    }
    //returns a double which shows the heading of the REV hub
    public static double heading(BNO055IMU imu) {
        Orientation angles;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return -AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }
    //returns the amount we should affect the motor powers by to re-balance the robot
    public double PI(double desiredHeading) {
        double error = Math.abs(desiredHeading - Math.abs(heading(imu)));
        double Kp = 0.01;
        if (error > 2.0 || error < -2.0) {
            return Kp * error;
        }
        else {
            return 0;
        }
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS , double desiredHeading) {
        int leftBackTarget;
        int rightBackTarget;
        int rightFrontTarget;
        int leftFrontTarget;
        //int averageTarget;
        double leftSpeed, rightSpeed;
        if (opModeIsActive()) {
            if(leftInches < 0){
                leftSpeed = speed*-1;
            }
            else {
                leftSpeed = speed;
            }
            if(rightInches < 0){
                rightSpeed = speed*-1;
            }
            else {
                rightSpeed = speed;
            }
            resetEncoders();
            // Determine new target position, and pass to motor controller
            leftBackTarget =  (leftBackMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH));
            rightBackTarget = (rightBackMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH));
            leftFrontTarget = (leftFrontMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH));
            rightFrontTarget = (rightFrontMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH));
            //averageTarget = ((Math.abs(leftBackTarget) + Math.abs(leftFrontTarget)
            //      +Math.abs(rightFrontTarget) + Math.abs(rightBackTarget))/4);
            leftFrontMotor.setTargetPosition(leftFrontTarget);
            leftBackMotor.setTargetPosition(leftBackTarget);
            rightFrontMotor.setTargetPosition(rightFrontTarget);
            rightBackMotor.setTargetPosition(rightBackTarget);



            // Turn On RUN_TO_POSITION
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            rightBackMotor.setPower((rightSpeed + PI(desiredHeading)));
            rightFrontMotor.setPower((rightSpeed + PI(desiredHeading)));
            leftFrontMotor.setPower((leftSpeed - PI(desiredHeading)));
            leftBackMotor.setPower((leftSpeed -  PI(desiredHeading)));

            //prints the desired position and actual position of all four motors
            //adjusts the motor powers according to the PI function
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)
                    &&(rightBackMotor.isBusy()) && (rightFrontMotor.isBusy()) && (leftBackMotor.isBusy()) && (leftFrontMotor.isBusy())) {
                telemetry.addData("Left Back Current Position", leftBackMotor.getCurrentPosition());
                telemetry.addData("Left Back Desired Position", leftBackTarget);
                telemetry.addData("Right Back Current Position", rightBackMotor.getCurrentPosition());
                telemetry.addData("Right Back Desired Position", rightBackTarget);
                telemetry.addData("Left Front Current Position", leftFrontMotor.getCurrentPosition());
                telemetry.addData("Left Front Desired Position", leftFrontTarget);
                telemetry.addData("Right Front Current Position", rightFrontMotor.getCurrentPosition());
                telemetry.addData("Right Front Desired Position", rightFrontTarget);
                //telemetry.addData("rightSpeed",rightSpeed);
                //telemetry.addData("leftSpeed",leftSpeed);
                telemetry.addData("heading", heading(imu));
                telemetry.update();
                leftSpeed = (accelerate(leftBackMotor,leftSpeed,leftBackTarget)+accelerate(leftFrontMotor,leftSpeed,leftFrontTarget)/2);
                rightSpeed = (accelerate(rightBackMotor,rightSpeed,rightBackTarget)+accelerate(rightFrontMotor,rightSpeed,rightFrontTarget)/2);
                rightBackMotor.setPower((rightSpeed + PI(desiredHeading)));
                rightFrontMotor.setPower((rightSpeed + PI(desiredHeading)));
                leftFrontMotor.setPower((leftSpeed - PI(desiredHeading)));
                leftBackMotor.setPower((leftSpeed -  PI(desiredHeading)));
            }

            leftBackMotor.setPower(0);
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightBackMotor.setPower(0);
            sleep(100);
        }
    }
    public void encoderStrafe(double speed,
                              double leftInches, double rightInches,
                              double timeoutS , double desiredHeading) {
        int leftBackTarget;
        int rightBackTarget;
        int rightFrontTarget;
        int leftFrontTarget;
        //int averageTarget;
        double leftSpeed, rightSpeed;
        if (opModeIsActive()) {
            if(leftInches < 0){
                leftSpeed = speed*-1;
            }
            else {
                leftSpeed = speed;
            }
            if(rightInches < 0){
                rightSpeed = speed*-1;
            }
            else {
                rightSpeed = speed;
            }
            resetEncoders();
            // Determine new target position, and pass to motor controller
            leftBackTarget =  (leftBackMotor.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH));
            rightBackTarget = (rightBackMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH));
            leftFrontTarget = (leftFrontMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH));
            rightFrontTarget = (rightFrontMotor.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH));
            //averageTarget = ((Math.abs(leftBackTarget) + Math.abs(leftFrontTarget)
            //      +Math.abs(rightFrontTarget) + Math.abs(rightBackTarget))/4);
            leftFrontMotor.setTargetPosition(leftFrontTarget);
            leftBackMotor.setTargetPosition(leftBackTarget);
            rightFrontMotor.setTargetPosition(rightFrontTarget);
            rightBackMotor.setTargetPosition(rightBackTarget);



            // Turn On RUN_TO_POSITION
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            rightBackMotor.setPower((rightSpeed + PI(desiredHeading)));
            rightFrontMotor.setPower((rightSpeed + PI(desiredHeading)));
            leftFrontMotor.setPower((leftSpeed - PI(desiredHeading)));
            leftBackMotor.setPower((leftSpeed -  PI(desiredHeading)));

            //prints the desired position and actual position of all four motors
            //adjusts the motor powers according to the PI function
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)
                    &&(rightBackMotor.isBusy()) && (rightFrontMotor.isBusy()) && (leftBackMotor.isBusy()) && (leftFrontMotor.isBusy())) {
                telemetry.addData("Left Back Current Position", leftBackMotor.getCurrentPosition());
                telemetry.addData("Left Back Desired Position", leftBackTarget);
                telemetry.addData("Right Back Current Position", rightBackMotor.getCurrentPosition());
                telemetry.addData("Right Back Desired Position", rightBackTarget);
                telemetry.addData("Left Front Current Position", leftFrontMotor.getCurrentPosition());
                telemetry.addData("Left Front Desired Position", leftFrontTarget);
                telemetry.addData("Right Front Current Position", rightFrontMotor.getCurrentPosition());
                telemetry.addData("Right Front Desired Position", rightFrontTarget);
                telemetry.addData("heading", heading(imu));
                //telemetry.addData("Average Target",averageTarget);
                telemetry.addData("rightSpeed",rightSpeed);
                telemetry.addData("leftSpeed",leftSpeed);
                telemetry.update();
                leftSpeed = (accelerate(leftBackMotor,leftSpeed,leftBackTarget)+accelerate(leftFrontMotor,leftSpeed,leftFrontTarget)/2);
                rightSpeed = (accelerate(rightBackMotor,rightSpeed,rightBackTarget)+accelerate(rightFrontMotor,rightSpeed,rightFrontTarget)/2);
                rightBackMotor.setPower((rightSpeed + PI(desiredHeading)));
                rightFrontMotor.setPower((rightSpeed + PI(desiredHeading)));
                leftFrontMotor.setPower((leftSpeed - PI(desiredHeading)));
                leftBackMotor.setPower((leftSpeed -  PI(desiredHeading)));
            }

            leftBackMotor.setPower(0);
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightBackMotor.setPower(0);
            sleep(100);
        }
    }
    public void resetEncoders(){
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void initAuto(){
        leftFrontMotor = hardwareMap.dcMotor.get("leftFrontDrive");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFrontDrive");
        leftBackMotor = hardwareMap.dcMotor.get("leftBackDrive");
        rightBackMotor = hardwareMap.dcMotor.get("rightBackDrive");
        armRotationMotor = hardwareMap.dcMotor.get("armRotationMotor");
        armExtensionMotor = hardwareMap.dcMotor.get("armExtensionMotor");

        armRotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double servoPosition = .31;
        double stoneServoPosition = 0;
        double grabberServoPosition = 1;

        leftServo = hardwareMap.servo.get("leftServo");
        rightServo = hardwareMap.servo.get("rightServo");
        stoneServo = hardwareMap.servo.get("stoneServo");
        grabberServo = hardwareMap.servo.get("grabberServo");

        //IMU Stuff, sets up parameters and reports accelerations to logcat log
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmodeaz
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);


        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public double accelerate(DcMotor motor, double speed, double target){
        if(motor.getCurrentPosition()<(target/10)){
            return speed*1.30;
        }
        else if(motor.getCurrentPosition()>(target*8/10)){
            return speed*0.9;
        }
        return speed;
    }
}
