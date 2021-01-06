/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp (name="Shooter Teleop", group="Pushbot")
//@Disabled
public class ShooterTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    IntakeMechanismHardwareMap robot   = new  IntakeMechanismHardwareMap ();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();


    static final double     MAX_SPEED = 1;
    static double IntakeSpeed;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        double elapTime = System.currentTimeMillis();
        robot.init(hardwareMap);
        double power = 0;
        double timeShooting = 0;
        double CurTime = elapTime;
        double LastTime = elapTime;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

/*
        // Step 1:  Drive forward for 3 seconds
        robot.leftfrontDrive.setPower(FORWARD_SPEED);
        robot.rightfrontDrive.setPower(FORWARD_SPEED);
        robot.leftbackDrive.setPower(FORWARD_SPEED);
        robot.rightbackDrive.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
        }
        if(gamepad1.a) {
            telemetry.addData("a is pressed", "indeed");
            telemetry.update();
        }
        else{
            telemetry.addData("a is not pressed", "indeed");
            telemetry.update();
            }
            */

        while (opModeIsActive()) {
            if(gamepad1.right_bumper==true) {
                double period = 5000;
                double quadScale = 0.1;
                CurTime = elapTime;
                timeShooting = timeShooting + (LastTime-CurTime);
                LastTime = CurTime;

                if(timeShooting > period) {
                    //power = -1 * (timeShooting / period);
                    //power = -quadScale * ((timeShooting/period) * (timeShooting/period));
                    power = Math.pow((timeShooting/period), 3);
                }
                else {
                    power = -1;
                }
            }
            else {
                timeShooting = 0;
                power = 0;
            }

            // Send calculated power to wheels
            robot.shooterMotor.setPower(power);

            IntakeSpeed=-gamepad1.right_stick_y;
            robot.intakeMotor.setPower(IntakeSpeed);
            robot.innerpastaMotor.setPower(IntakeSpeed);

            // Show the elapsed game time and wheel power.
        }

    }
}
