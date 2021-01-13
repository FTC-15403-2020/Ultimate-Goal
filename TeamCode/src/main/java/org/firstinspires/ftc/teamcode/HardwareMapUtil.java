package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public abstract class HardwareMapUtil {
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();




    public DcMotor HardwareInitMotor(String configname, boolean forward) {
        DcMotor motor = null;
        motor = hwMap.get(DcMotor.class, configname);
        motor.setPower(0);
        //If the motor is a wheel on the left of the robot OR another motor somewhere else use FORWARD!
        if (forward) {
            motor.setDirection(DcMotor.Direction.FORWARD);
        }
        //If the motor is a wheel on the right of the robot then use REVERSE!
        else {
            motor.setDirection(DcMotor.Direction.REVERSE);
        }
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;

    }
    public Servo HardwareInitServo(String configname, double position){
        Servo servo = null;
        servo = hwMap.get(Servo.class, configname);
        servo.setPosition(position);
        return servo;
    }
    public CRServo HardwareInitCRServo(String configname, boolean forward){
        CRServo crservo = null;
        crservo = hardwareMap.crservo.get(configname);
        crservo.setPower(0);
        if(forward){
            crservo.setDirection(CRServo.Direction.FORWARD);
        }
        else{
            crservo.setDirection(CRServo.Direction.REVERSE);
        }
        return crservo;
    }
    public ColorSensor HardwareInitColorSensor(String configname){
        return hwMap.get(ColorSensor.class, configname);
        //UNTESTED!
    }

}
