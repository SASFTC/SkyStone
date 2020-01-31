package org.firstinspires.ftc.team17156;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team17156.scotslib.hardware.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.team17156.scotslib.hardware.extension.Intake;
import org.firstinspires.ftc.team17156.scotslib.hardware.extension.LiftingSystem;
import org.firstinspires.ftc.team17156.scotslib.hardware.extension.TurnAngle;

@TeleOp(name = "servo_testing", group = "Test")
public class servo_testing extends OpMode {


    /* Fields */
    private ElapsedTime runtime = new ElapsedTime();
    private Servo testingServo;
    private double testValue = 0.5;
    private boolean isLeftRelease = false;
    private boolean isRightRelease = false;
    private boolean isaLeftRelease = false;
    private boolean isaRightRelease = false;



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
        this.testingServo = hardwareMap.get(Servo.class, "wrist_servo");
        this.testingServo.scaleRange(0, 1);


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
//        this.testingMotor.turn(1.0, 0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if (gamepad1.left_bumper && !isLeftRelease){
            isLeftRelease = true;
            testValue -= 0.01;
        } else if (gamepad1.right_bumper && !isRightRelease){
            isRightRelease = true;
            testValue += 0.01;
        } else if (!gamepad1.right_bumper && !gamepad1.left_bumper) {
            isRightRelease = false;
            isLeftRelease = false;
        }
        if (isPressed(gamepad1.left_trigger) && !isPressed(gamepad1.right_trigger) && !isaLeftRelease){
            isaLeftRelease = true;
            testValue -= 0.1;
        } else if (isPressed(gamepad1.right_trigger) && !isPressed(gamepad1.left_trigger) && !isaRightRelease){
            isaRightRelease = true;
            testValue += 0.1;
        } else if (!isPressed(gamepad1.right_trigger) && !isPressed(gamepad1.left_trigger)) {
            isaRightRelease = false;
            isaLeftRelease = false;
        }
        telemetry.addData("value", testValue);
        telemetry.update();
        testingServo.setPosition(testValue);

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

}