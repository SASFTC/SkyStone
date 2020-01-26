package org.firstinspires.ftc.team17156;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team17156.scotslib.hardware.extension.LiftingSystem;

@TeleOp(name = "Component Testing", group = "Testing")
public class Component_Testing extends OpMode {

    LiftingSystem liftingSystem;

    @Override
    public void init() {

        this.liftingSystem = new LiftingSystem(hardwareMap, "lifting_motor",
                "wrist_servo", "grabbing_servo", 1);


    }

    @Override
    public void loop() {


        this.liftingSystem.raise(-gamepad1.left_stick_y);

        if (gamepad1.a && !gamepad1.b) {
            this.liftingSystem.grabber(LiftingSystem.Grabber.GRAB);
        } else if (gamepad1.b && !gamepad1.a) {
            this.liftingSystem.grabber(LiftingSystem.Grabber.RELEASE);
        }

        if (gamepad1.dpad_up && !gamepad1.dpad_down) {
            this.liftingSystem.swingWrist(LiftingSystem.Direction.OUT);
        } else if (gamepad1.dpad_down && !gamepad1.dpad_up) {
            this.liftingSystem.swingWrist(LiftingSystem.Direction.IN);
        }


    }
}
