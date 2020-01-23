// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
// This example assumes there is one encoder, attached to the left motor.

package org.firstinspires.ftc.team17156;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

@TeleOp(name="Drive Encoder", group="Exercises")
//@Disabled
public class encoder extends LinearOpMode
{
    DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException
    {
        motor = hardwareMap.dcMotor.get("right_motor");


        // reset encoder count kept by left motor.

        // set left motor to run to target encoder position and stop with brakes on.
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // wait for start button.

        waitForStart();

        // set left motor to run to 5000 encoder counts.

        motor.setTargetPosition(5000);

        // set both motors to 25% power. Movement will start.
        motor.setPower(0.25);

        // wait while opmode is active and left motor is busy running to position.

        while (opModeIsActive() && motor.isBusy())
        {
            telemetry.addData("encoder-fwd", leftMotor.getCurrentPosition() + "  busy=" + motor.isBusy());
            telemetry.update();
            idle();
        }

        // set motor power to zero to turn off motors. The motors stop on their own but
        // power is still applied so we turn off the power.

        motor.setPower(0.0);

        // wait 5 sec so you can observe the final encoder position.

        resetStartTime();

        while (opModeIsActive() && getRuntime() < 5)
        {
            telemetry.addData("encoder-fwd-end", leftMotor.getCurrentPosition() + "  busy=" + motor.isBusy());
            telemetry.update();
            idle();
        }

        // Now back up to starting point. In this example instead of
        // having the motor monitor the encoder, we will monitor the encoder ourselves.

        while (opModeIsActive() && motor.getCurrentPosition() > 0)
        {
            telemetry.addData("encoder-back", motor.getCurrentPosition());
            telemetry.update();
            idle();
        }

        // set motor power to zero to stop motors.

        // wait 5 sec so you can observe the final encoder position.

        resetStartTime();

        while (opModeIsActive() && getRuntime() < 5)
        {
            telemetry.addData("encoder-back-end", leftMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }
}