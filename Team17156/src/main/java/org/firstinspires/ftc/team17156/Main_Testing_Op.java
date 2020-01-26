package org.firstinspires.ftc.team17156;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team17156.scotslib.hardware.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.team17156.scotslib.hardware.extension.FoundationHolder;
import org.firstinspires.ftc.team17156.scotslib.hardware.extension.Intake;
import org.firstinspires.ftc.team17156.scotslib.hardware.extension.LiftingSystem;

@TeleOp(name = "Main Testing Op", group = "Test")
public class Main_Testing_Op extends OpMode {


    /* Fields */
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumDrivetrain dTrain;
    private Intake intake;
    private LiftingSystem liftingSystem;
    private FoundationHolder foundationHolder;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Initialize all components.
        this.dTrain = new MecanumDrivetrain(hardwareMap, "front_left_motor",
                "front_right_motor", "back_left_motor",
                "back_right_motor", 100, 1);

        this.intake = new Intake(hardwareMap, "left_intake_motor",
                "right_intake_motor", 1);

        this.liftingSystem = new LiftingSystem(hardwareMap, "lifting_motor",
                "wrist_servo", "grabbing_servo", 1);

        this.foundationHolder = new FoundationHolder(hardwareMap, "left_foundation_servo",
                "right_foundation_servo");

        // Telemetry.
        telemetry.addData("Status", "Initialized");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // Drivetrain.
        this.dTrain.driveJoystick(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        // Intake.
        if (gamepad1.a && !gamepad1.b) {
            this.intake.run(Intake.Direction.IN);
        } else if (gamepad1.b && !gamepad1.a) {
            this.intake.run(Intake.Direction.OUT);
        } else {
            this.intake.run(Intake.Direction.STOP);
        }

        // Lifting System.
        if (gamepad1.dpad_up && !gamepad1.dpad_down) {
            this.liftingSystem.raise(1);
        } else if (gamepad1.dpad_down && !gamepad1.dpad_up) {
            this.liftingSystem.raise(-1);
        } else {
            this.liftingSystem.stop();
        }

        // Foundation Holder.
        if (gamepad1.x && !gamepad1.y) {
            this.foundationHolder.grab();
        } else if (gamepad1.y && !gamepad1.x) {
            this.foundationHolder.release();
        }

        // Telemetry.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}
