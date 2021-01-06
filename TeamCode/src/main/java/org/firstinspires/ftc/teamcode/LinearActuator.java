package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name="LinearActuator",group="hello")
public class LinearActuator extends LinearOpMode {
    ShooterHardwareMap robot = new ShooterHardwareMap();
    private ElapsedTime runtime = new ElapsedTime();

    static double actuator_power = 0.5;



    public void runOpMode(){

        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
           // actuator_power=-gamepad1.left_stick_y;
           //robot.linearActuator.setPower(actuator_power);
            }

        }

    }



