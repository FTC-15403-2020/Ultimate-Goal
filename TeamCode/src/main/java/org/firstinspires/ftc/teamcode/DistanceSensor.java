package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp

public class DistanceSensor extends LinearOpMode {

    private DistanceSensor sensorRange;

    @Override
    public void runOpMode(){
        sensorRange= hardwareMap.get(DistanceSensor.class, "sensor_range");

    }
}
