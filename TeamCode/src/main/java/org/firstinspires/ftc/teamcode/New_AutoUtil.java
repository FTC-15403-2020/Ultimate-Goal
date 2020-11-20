package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Auto_Util", group="abstract")
@Disabled
public class New_AutoUtil extends LinearOpMode{
    //Drive motors
    DcMotor rfmotor, rbmotor, lfmotor, lbmotor;
    //Utility motors
    DcMotor utilmotor1, utilmotor2, utilmotor3, utilmotor4;
    //odometry encoders
    DcMotor verticalLeft, verticalRight, horizontal;
    private ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;
    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "rightFrontDrive", rbName = "rightBackDrive", lfName = "leftFrontDrive", lbName = "leftBackDrive";
    String verticalLeftEncoderName = lbName, verticalRightEncoderName = lfName, horizontalEncoderName = rfName;
    OdometryGlobalCoordinatePosition globalPositionUpdate;
    final double COUNTS_PER_INCH = 307.699557;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("you shouldn't be here!", "Silly!");
        telemetry.update();
    }

    public void assignDriveBase(DcMotor rightfrontmotor, DcMotor rightbackmotor, DcMotor leftfrontmotor, DcMotor leftbackmotor){
        rfmotor = rightfrontmotor; rbmotor = rightbackmotor; lfmotor = leftfrontmotor; lbmotor = leftbackmotor;
    }
    public void assignUtilMotors(DcMotor util1, DcMotor util2, DcMotor util3, DcMotor util4){
        utilmotor1 = util1; utilmotor2 = util2; utilmotor3 = util3; utilmotor4 = util4;
    }
    public void setAllDriveMotors(double time){
        runtime.reset();
        while(runtime.seconds() < time){
            rfmotor.setPower(1); rbmotor.setPower(1);
            lfmotor.setPower(1); lbmotor.setPower(1);
        }
    }
    public void strafeLeft(double time){
        runtime.reset();
        while(runtime.seconds() < time){
            rfmotor.setPower(1); rbmotor.setPower(-1);
            lfmotor.setPower(-1); lbmotor.setPower(1);
        }
    }
    public void strafeRight(double time){
        runtime.reset();
        while(runtime.seconds() < time){
            rfmotor.setPower(-1); rbmotor.setPower(1);
            lfmotor.setPower(1); lbmotor.setPower(-1);
        }
    }
    public void turnRight(double time){
        runtime.reset();
        while(runtime.seconds() < time){
            rfmotor.setPower(-1); rbmotor.setPower(-1);
            lfmotor.setPower(1); lbmotor.setPower(1);
        }
    }
    public void turnLeft(double time){
        runtime.reset();
        while(runtime.seconds() < time){
            rfmotor.setPower(1); rbmotor.setPower(1);
            lfmotor.setPower(-1); lbmotor.setPower(-1);
        }
    }
    public static double heading(BNO055IMU imu) {
        Orientation angles;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return -AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }
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
    public void initOdometry(){
        //Initialize hardware map values.
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);
        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();
        globalPositionUpdate.reverseLeftEncoder();
    }
    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        rfmotor = hardwareMap.dcMotor.get(rfName);
        rbmotor = hardwareMap.dcMotor.get(rbName);
        lfmotor = hardwareMap.dcMotor.get(lfName);
        lbmotor = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        rfmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lfmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rfmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rbmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lfmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lbmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        rfmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lfmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}

