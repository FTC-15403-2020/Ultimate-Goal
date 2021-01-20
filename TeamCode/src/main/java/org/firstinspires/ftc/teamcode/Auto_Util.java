package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Auto_Util", group="abstract")
@Disabled
public abstract class Auto_Util extends LinearOpMode{
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double ENCODER_COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159);
    static final double DRIVE_SPEED = 0.2;
    static final double STRAFE_SPEED = 0.8;
    //Drive motors
    DcMotor rfmotor, rbmotor, lfmotor, lbmotor;
    //Utility motors
    DcMotor utilmotor1, utilmotor2, utilmotor3, utilmotor4;
    //odometry encoders
    DcMotor verticalLeft, verticalRight, horizontal;
    //servos
    Servo servo1;
    CRServo crservo1, crservo2;
    private ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;
    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "rfD", rbName = "rbD", lfName = "lfD", lbName = "lbD";
    String util1name = "Intake", util2name = "pastaM", util3name = "shootM", util4name = "wobbleG";
    String servo1name = "wobbleS", crservo1name = "pastaS", crservo2name = "pastaS2";
    String verticalLeftEncoderName = lbName, verticalRightEncoderName = lfName, horizontalEncoderName = rfName;
    OdometryGlobalCoordinatePosition globalPositionUpdate;
    final double ODOMETRY_COUNTS_PER_INCH = 307.699557;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("you shouldn't be here!", "This program isnt meant to be run, only for use with all of its methods");
        telemetry.update();
    }

    public void assignDriveBase(DcMotor rightfrontmotor, DcMotor rightbackmotor, DcMotor leftfrontmotor, DcMotor leftbackmotor){
        rfmotor = rightfrontmotor; rbmotor = rightbackmotor; lfmotor = leftfrontmotor; lbmotor = leftbackmotor;
    }
    public void assignUtilMotors(DcMotor util1, DcMotor util2, DcMotor util3, DcMotor util4){
        utilmotor1 = util1; utilmotor2 = util2; utilmotor3 = util3; utilmotor4 = util4;
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
    /*public void initOdometry(){
        //Initialize hardware map values.
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);
        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, ODOMETRY_COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();
        globalPositionUpdate.reverseLeftEncoder();
    }
     */
    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName){
        rfmotor = hardwareMap.dcMotor.get(rfName);
        rbmotor = hardwareMap.dcMotor.get(rbName);
        lfmotor = hardwareMap.dcMotor.get(lfName);
        lbmotor = hardwareMap.dcMotor.get(lbName);

        /*verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);
         */

        rfmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lfmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rfmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rbmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lfmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lbmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lbmotor.setDirection(DcMotor.Direction.REVERSE);
        lfmotor.setDirection(DcMotor.Direction.REVERSE);
        rbmotor.setDirection(DcMotor.Direction.FORWARD);
        rfmotor.setDirection(DcMotor.Direction.FORWARD);

        /*verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         */

        /*verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

         */


        rfmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lfmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    private void initUtilHardwareMap(String util1name, String util2name, String util3name, String util4name){
        utilmotor1 = hardwareMap.dcMotor.get(util1name);
        utilmotor2 = hardwareMap.dcMotor.get(util2name);
        utilmotor3 = hardwareMap.dcMotor.get(util3name);
        utilmotor4 = hardwareMap.dcMotor.get(util4name);

        utilmotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        utilmotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        utilmotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        utilmotor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        utilmotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        utilmotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        utilmotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        utilmotor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        utilmotor1.setDirection(DcMotor.Direction.FORWARD);
        utilmotor2.setDirection(DcMotor.Direction.FORWARD);
        utilmotor3.setDirection(DcMotor.Direction.FORWARD);
        utilmotor4.setDirection(DcMotor.Direction.FORWARD);

        utilmotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        utilmotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        utilmotor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        utilmotor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    private void initServoHardwareMap(String servo1name, String crservo1name, String crservo2name){
        servo1 = hardwareMap.servo.get(servo1name);
        servo1.setPosition(0);
        crservo1 = hardwareMap.crservo.get(crservo1name);
        crservo2 = hardwareMap.crservo.get(crservo2name);
        crservo1.setDirection(CRServo.Direction.FORWARD);
        crservo2.setDirection(CRServo.Direction.FORWARD);
        crservo1.setPower(0);
        crservo2.setPower(0);
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
            leftBackTarget =  (lbmotor.getCurrentPosition() + (int) (leftInches * ENCODER_COUNTS_PER_INCH));
            rightBackTarget = (rbmotor.getCurrentPosition() + (int) (rightInches * ENCODER_COUNTS_PER_INCH));
            leftFrontTarget = (lfmotor.getCurrentPosition() + (int) (leftInches * ENCODER_COUNTS_PER_INCH));
            rightFrontTarget = (rfmotor.getCurrentPosition() + (int) (rightInches * ENCODER_COUNTS_PER_INCH));
            //averageTarget = ((Math.abs(leftBackTarget) + Math.abs(leftFrontTarget)
            //      +Math.abs(rightFrontTarget) + Math.abs(rightBackTarget))/4);
            lfmotor.setTargetPosition(leftFrontTarget);
            lbmotor.setTargetPosition(leftBackTarget);
            rfmotor.setTargetPosition(rightFrontTarget);
            rbmotor.setTargetPosition(rightBackTarget);



            // Turn On RUN_TO_POSITION
            rfmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rbmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lbmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lfmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            rbmotor.setPower((rightSpeed + PI(desiredHeading)));
            rfmotor.setPower((rightSpeed + PI(desiredHeading)));
            lfmotor.setPower((leftSpeed - PI(desiredHeading)));
            lbmotor.setPower((leftSpeed -  PI(desiredHeading)));

            //prints the desired position and actual position of all four motors
            //adjusts the motor powers according to the PI function
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)
                    &&(rbmotor.isBusy()) && (rfmotor.isBusy()) && (lbmotor.isBusy()) && (lfmotor.isBusy())) {
                telemetry.addData("Left Back Current Position", lbmotor.getCurrentPosition());
                telemetry.addData("Left Back Desired Position", leftBackTarget);
                telemetry.addData("Right Back Current Position", rbmotor.getCurrentPosition());
                telemetry.addData("Right Back Desired Position", rightBackTarget);
                telemetry.addData("Left Front Current Position", lfmotor.getCurrentPosition());
                telemetry.addData("Left Front Desired Position", leftFrontTarget);
                telemetry.addData("Right Front Current Position", rfmotor.getCurrentPosition());
                telemetry.addData("Right Front Desired Position", rightFrontTarget);
                //telemetry.addData("rightSpeed",rightSpeed);
                //telemetry.addData("leftSpeed",leftSpeed);
                telemetry.addData("heading", heading(imu));
                telemetry.update();
                leftSpeed = (accelerate(lbmotor,leftSpeed,leftBackTarget)+accelerate(lfmotor,leftSpeed,leftFrontTarget)/2);
                rightSpeed = (accelerate(rbmotor,rightSpeed,rightBackTarget)+accelerate(rfmotor,rightSpeed,rightFrontTarget)/2);
                rbmotor.setPower((rightSpeed + PI(desiredHeading)));
                rfmotor.setPower((rightSpeed + PI(desiredHeading)));
                lfmotor.setPower((leftSpeed - PI(desiredHeading)));
                lbmotor.setPower((leftSpeed -  PI(desiredHeading)));
            }

            lbmotor.setPower(0);
            lfmotor.setPower(0);
            rfmotor.setPower(0);
            rbmotor.setPower(0);
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
            leftBackTarget =  (lbmotor.getCurrentPosition() - (int) (leftInches * ENCODER_COUNTS_PER_INCH));
            rightBackTarget = (rbmotor.getCurrentPosition() + (int) (rightInches * ENCODER_COUNTS_PER_INCH));
            leftFrontTarget = (lfmotor.getCurrentPosition() + (int) (leftInches * ENCODER_COUNTS_PER_INCH));
            rightFrontTarget = (rfmotor.getCurrentPosition() - (int) (rightInches * ENCODER_COUNTS_PER_INCH));
            //averageTarget = ((Math.abs(leftBackTarget) + Math.abs(leftFrontTarget)
            //      +Math.abs(rightFrontTarget) + Math.abs(rightBackTarget))/4);
            lfmotor.setTargetPosition(leftFrontTarget);
            lbmotor.setTargetPosition(leftBackTarget);
            rfmotor.setTargetPosition(rightFrontTarget);
            rbmotor.setTargetPosition(rightBackTarget);



            // Turn On RUN_TO_POSITION
            rfmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rbmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lbmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lfmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            rbmotor.setPower((rightSpeed + PI(desiredHeading)));
            rfmotor.setPower((rightSpeed + PI(desiredHeading)));
            lfmotor.setPower((leftSpeed - PI(desiredHeading)));
            lbmotor.setPower((leftSpeed -  PI(desiredHeading)));

            //prints the desired position and actual position of all four motors
            //adjusts the motor powers according to the PI function
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)
                    &&(rbmotor.isBusy()) && (rfmotor.isBusy()) && (lbmotor.isBusy()) && (lfmotor.isBusy())) {
                telemetry.addData("Left Back Current Position", lbmotor.getCurrentPosition());
                telemetry.addData("Left Back Desired Position", leftBackTarget);
                telemetry.addData("Right Back Current Position", rbmotor.getCurrentPosition());
                telemetry.addData("Right Back Desired Position", rightBackTarget);
                telemetry.addData("Left Front Current Position", lfmotor.getCurrentPosition());
                telemetry.addData("Left Front Desired Position", leftFrontTarget);
                telemetry.addData("Right Front Current Position", rfmotor.getCurrentPosition());
                telemetry.addData("Right Front Desired Position", rightFrontTarget);
                telemetry.addData("heading", heading(imu));
                //telemetry.addData("Average Target",averageTarget);
                telemetry.addData("rightSpeed",rightSpeed);
                telemetry.addData("leftSpeed",leftSpeed);
                telemetry.update();
                leftSpeed = (accelerate(lbmotor,leftSpeed,leftBackTarget)+accelerate(lfmotor,leftSpeed,leftFrontTarget)/2);
                rightSpeed = (accelerate(rbmotor,rightSpeed,rightBackTarget)+accelerate(rfmotor,rightSpeed,rightFrontTarget)/2);
                rbmotor.setPower((rightSpeed + PI(desiredHeading)));
                rfmotor.setPower((rightSpeed + PI(desiredHeading)));
                lfmotor.setPower((leftSpeed - PI(desiredHeading)));
                lbmotor.setPower((leftSpeed -  PI(desiredHeading)));
            }

            lbmotor.setPower(0);
            lfmotor.setPower(0);
            rfmotor.setPower(0);
            rbmotor.setPower(0);
            sleep(100);
        }
    }
    public void resetEncoders(){
        lfmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lfmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*
    lfD
    lbD
    rbD
    rfD
    Intake
    pastaM
    shootM
    wobbleG

    Servos:

    wobbleS
    pastaS
    pastaS2

     String verticalLeftEncoderName = lbName, verticalRightEncoderName = lfName, horizontalEncoderName = rfName;
     */




     public void initAuto(){
         initDriveHardwareMap(rfName, rbName, lfName, lbName);
         initUtilHardwareMap(util1name, util2name, util3name, util4name);
         initServoHardwareMap(servo1name, crservo1name, crservo2name);
         //IMU Stuff, sets up parameters and reports accelerations to logcat log
         BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
         parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
         parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
         parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmodeaz
         imu = hardwareMap.get(BNO055IMU.class, "imu");
         imu.initialize(parameters);
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
}

