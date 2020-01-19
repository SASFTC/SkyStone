package org.firstinspires.ftc.team17156;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team17156.scotslib.hardware.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.team17156.scotslib.hardware.extension.Intake;

@TeleOp(name = "Main Testing Op", group = "Test")
public class Main_Testing_Op extends OpMode {


    /* Fields */
    MecanumDrivetrain dTrain;
    Intake intake;

    @Override
    public void init() {

        // Initialize all components.
        this.dTrain = new MecanumDrivetrain(hardwareMap, "front_left_motor",
                "front_right_motor", "back_left_motor",
                "back_right_motor", 100, 1, false);

        this.intake = new Intake(hardwareMap, "left_intake_motor",
                "right_intake_motor", 1);



    }

    @Override
    public void loop() {

    }
}
