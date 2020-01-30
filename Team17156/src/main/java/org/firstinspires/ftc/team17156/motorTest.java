package org.firstinspires.ftc.team17156;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team17156.scotslib.hardware.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.team17156.scotslib.hardware.extension.Intake;
import org.firstinspires.ftc.team17156.scotslib.hardware.extension.IntakeServo;
import org.firstinspires.ftc.team17156.scotslib.hardware.extension.LiftingSystem;
import org.firstinspires.ftc.team17156.scotslib.hardware.extension.TurnAngle;

import static java.lang.Thread.sleep;

@TeleOp(name = "Motor test", group = "Test")
public class motorTest extends OpMode {


    /* Fields */
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumDrivetrain machanumDrive;
    private Intake intake;
    private LiftingSystem liftingSystem;
    private DcMotor testingMotor;
    private boolean isRightTriggerReleased = true;
    private boolean isLeftTriggerReleased = true;
    private LiftingSystem liftingSys;
    private double speedFactor = 0.6;
    private boolean isStickPressed = false;
    private IntakeServo intakeServo;
    private double rotationFactor = 0.6;
    private double intakeServoOut = 0.73;
    private double intakeServoIn = 0.16;
    private int currentBricks = 0;
    private Intake.Direction rotationDirection = Intake.Direction.STOP;
    private boolean firstTimeLoop = true;
    private double angleIncrement = 0;
    private TurnAngle angleMotor;

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

        testingMotor = hardwareMap.get(DcMotor.class, "lifting_motor");
        angleMotor = new TurnAngle(hardwareMap, this.testingMotor, 1, 100, 537.6);


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
        if (isPressed(gamepad1.right_trigger) && !isPressed(gamepad1.left_trigger) && isRightTriggerReleased){
            isRightTriggerReleased = false;
            angleIncrement += 50;
            angleMotor.turnTo(0.4, 50);
        } else if (isPressed(gamepad1.left_trigger) && !isPressed(gamepad1.right_trigger)){
            isLeftTriggerReleased = false;
            angleIncrement -= 50;
            angleMotor.turnTo(0.4, -50);
        } else if (!isPressed(gamepad1.left_trigger) && !isPressed(gamepad1.right_trigger)){
            isLeftTriggerReleased = true;
            isRightTriggerReleased = true;
        }

        telemetry.addData("data", angleIncrement);
        telemetry.addData("pos", angleMotor.getMotorPosition());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

    public boolean isPressed(float triggerValue) {
        return triggerValue > 0.2;
    }

    public int isSwang(float stickValue) {
        if (stickValue < -0.2){
            return 1;
        } else if (stickValue > 0.2) {
            return -1;
        } else {
            return 0;
        }
    }
}