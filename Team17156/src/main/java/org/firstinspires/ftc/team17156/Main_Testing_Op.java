package org.firstinspires.ftc.team17156;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team17156.scotslib.hardware.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.team17156.scotslib.hardware.extension.Intake;
import org.firstinspires.ftc.team17156.scotslib.hardware.extension.LiftingSystem;
import org.firstinspires.ftc.team17156.scotslib.hardware.extension.TurnAngle;

@TeleOp(name = "Main Testing Op", group = "Test")
public class Main_Testing_Op extends OpMode {


    /* Fields */
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumDrivetrain dTrain;
    private Intake intake;
    private LiftingSystem liftingSystem;
    private TurnAngle testingMotor;
    private boolean isRightTriggerReleased = true;
    private boolean isLeftTriggerReleased = true;
    private LiftingSystem liftingSys;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Initialize all components.
//        this.dTrain = new MecanumDrivetrain(hardwareMap, "front_left_motor",
//                "front_right_motor", "back_left_motor",
//                "back_right_motor", 100, 1, false);
//
//        this.intake = new Intake(hardwareMap, "left_intake_motor",
//                "right_intake_motor", 1);
//
//        this.liftingSystem = new LiftingSystem(hardwareMap, "lifting_motor",
//                "wrist_servo", "grabbing_servo", 1, 50);

        // Telemetry.

        telemetry.addData("Status", "Initialized");
        liftingSys = new LiftingSystem(hardwareMap, "lifting_motor", 1.0, 100, 537.6);


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
//        this.testingMotor.turn(0.25, 90);
//        this.testingMotor = new TurnAngle(hardwareMap, "testing_motor", 1.0, 100, 1680);
        runtime.reset();
//        this.testingMotor.turnTo(1.0, 0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
//        telemetry.addData("position", Double.toString(liftingSys.getMotorPosition()));
        if ()
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
    public boolean isPressed(float triggerValue){
        return triggerValue > 0.2;
    }
}