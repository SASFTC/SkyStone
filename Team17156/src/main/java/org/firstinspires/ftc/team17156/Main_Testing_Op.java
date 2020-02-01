package org.firstinspires.ftc.team17156;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team17156.scotslib.hardware.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.team17156.scotslib.hardware.extension.CapstoneServo;
import org.firstinspires.ftc.team17156.scotslib.hardware.extension.Intake;
import org.firstinspires.ftc.team17156.scotslib.hardware.extension.IntakeServo;
import org.firstinspires.ftc.team17156.scotslib.hardware.extension.LiftingSystem;
import org.firstinspires.ftc.team17156.scotslib.hardware.extension.FoundationHolder;
import org.firstinspires.ftc.team17156.scotslib.hardware.extension.TurnAngle;
import org.firstinspires.ftc.team17156.scotslib.hardware.extension.sideGrabber;
import org.firstinspires.ftc.team17156.scotslib.hardware.ScotsHardware;

import static java.lang.Thread.sleep;

@TeleOp(name = "Main Testing Op", group = "Test")
public class Main_Testing_Op extends OpMode {


    /* Fields */
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumDrivetrain machanumDrive;
    private Intake intake;
    private sideGrabber sideGrabberObject;
    private LiftingSystem liftingSystem;
    private TurnAngle testingMotor;
    private boolean isRightTriggerReleased = true;
    private boolean isLeftTriggerReleased = true;
    private LiftingSystem liftingSys;
    private double speedFactor = 0.6;
    private boolean isStickPressed = false;
    private IntakeServo intakeServo;
    private FoundationHolder foundationHolder;
    private CapstoneServo capstoneServo;
    private double rotationFactor = 0.6;
    private double intakeServoOut = 0.73;
    private double intakeServoIn = 0.16;
    private int assignedBricks = 0;
    private Intake.Direction rotationDirection = Intake.Direction.STOP;
    private boolean firstTimeLoop = true;
    private Thread liftingThread1Object;
    private Thread liftingThread2Object;
    private Thread liftingThread3Object;
    private int addedValue = 0;
    private int maxBricks = 6;
    private TurnAngle angleMotor;
    private boolean isStickReleased = true;
    private ScotsHardware robot;
    private boolean firstTimeToRun = true;
    private DcMotor sideLifting;
    private int intakeDirection = 0;
    private boolean isStickClickReleased = true;
    private int phase = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        sideLifting = hardwareMap.get(DcMotor.class, "side_lifting_motor");
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
        robot = new ScotsHardware();
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");

        liftingSys = new LiftingSystem(hardwareMap, "lifting_motor", "wrist_servo", "grabbing_servo", 1.0, 100, 537.6);
        machanumDrive = new MecanumDrivetrain(hardwareMap, robot.leftFrontDrive,
                robot.rightFrontDrive, robot.leftBackDrive,
                robot.rightBackDrive, 100, 1, true);
        this.intake = new Intake(hardwareMap, "left_intake_motor",
                "right_intake_motor", 1);
        this.intakeServo = new IntakeServo(hardwareMap, "intake_servo");
        this.capstoneServo = new CapstoneServo(hardwareMap, "capstone_servo");
        this.sideGrabberObject = new sideGrabber(hardwareMap, "side_wrist_servo", "side_grabbing_servo");
        this.foundationHolder = new FoundationHolder(hardwareMap, "left_foundation_servo", "right_foundation_servo");
//        this.intakeServo.getServo().setPosition(this.intakeServoIn);
//        liftingThreadObject = new Thread(new liftingThread(liftingSys, intakeServo, 0));
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
//        this.testingMotor.turn(1.0, 0);
//        liftingThreadObject.start();
//        capstoneServo.getServo().setPosition(0.7);
//        try {
//            sleep(200);
//        } catch (InterruptedException e) {
//        }
//        capstoneServo.getServo().setPosition(0.15);
//        machanumDrive.driveByDistance(1.0, 0, 0, 10000);
//        angleMotor = new TurnAngle(hardwareMap, "front_left_motor", 1, 1.0, 537.6);
//        angleMotor.turn(1.0, 360);
//        sideGrabberObject.run(sideGrabber.State.DEFAULT);
//        this.foundationHolder.run(FoundationHolder.State.GRASP);
//        this.capstoneServo.run();
//        liftingSys.grabber(LiftingSystem.Grabber.GRAB);
//        liftingSys.goBricks(1.0, 3, true);
        liftingThread1Object = new Thread(new liftingThread1(liftingSys, intakeServo));
        liftingThread2Object = new Thread(new liftingThread2(liftingSys, intakeServo, 2));
        liftingThread3Object = new Thread(new liftingThread3(liftingSys, intakeServo, 2));
//        liftingThreadObject.start();
    }


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if (isPressed(gamepad1.right_trigger) && !isPressed(gamepad1.left_trigger) && isRightTriggerReleased) {
            isRightTriggerReleased = false;
            if (intakeDirection != 1)
                intakeDirection = 1;
            else
                intakeDirection = 0;
        } else if (isPressed(gamepad1.left_trigger) && !isPressed(gamepad1.right_trigger) && isLeftTriggerReleased){
            isLeftTriggerReleased = false;
            if (intakeDirection != -1)
                intakeDirection = -1;
            else
                intakeDirection = 0;
        } else if (!isPressed(gamepad1.right_trigger) && !isPressed(gamepad1.left_trigger)){
            isLeftTriggerReleased = true;
            isRightTriggerReleased = true;
        }
        if (intakeDirection == 0)
            intake.run(Intake.Direction.STOP);
        else if (intakeDirection == 1)
            intake.run(Intake.Direction.IN);
        else if (intakeDirection == -1)
            intake.run(Intake.Direction.OUT);
        if (isPressed(gamepad2.right_trigger)){
            sideLifting.setPower(1);
        } else if (isPressed(gamepad2.left_trigger)){
            sideLifting.setPower(-1);
        } else {
            sideLifting.setPower(0);
        }
////        telemetry.addData("pos", angleMotor.getMotorPosition());
////        telemetry.addData("position", Double.toString(liftingSys.getMotorPosition()));
//        if (isPressed(gamepad2.right_trigger) && !isPressed(gamepad2.left_trigger)) {
//            isLeftTriggerReleased = false;
//            isRightTriggerReleased = true;
//            liftingSys.goBricks(0.5, 1, true);
//        } else if (isPressed(gamepad2.left_trigger) && !isPressed(gamepad2.right_trigger)) {
//            isRightTriggerReleased = false;
//            isLeftTriggerReleased = true;
//            liftingSys.goBricks(0.5, -1, false);
//        } else {
//            isRightTriggerReleased = true;
//            isLeftTriggerReleased = true;
//        }
//
//        if (isPressed(gamepad1.right_trigger) && !isPressed(gamepad1.left_trigger)) {
//            intake.run(Intake.Direction.IN);
//        } else if (isPressed(gamepad1.left_trigger) && !isPressed(gamepad1.right_trigger)) {
//            intake.run(Intake.Direction.OUT);
//        } else {
//            intake.run(Intake.Direction.STOP);
//        }
//        if (gamepad2.right_bumper && !gamepad2.left_bumper) {
//            intakeServo.getServo().setPosition(intakeServoOut);
//            intakeServo.getServo().setPosition(intakeServoIn);
//            liftingSys.grabber(LiftingSystem.Grabber.GRAB);
//            liftingSys.swingWrist(LiftingSystem.Direction.OUT);
//        }
////        if (isSwang(gamepad2.right_stick_y) == 1) {
////            intakeServo.getServo().setPosition(intakeServoOut);
////        } else if (isSwang(gamepad2.right_stick_y) == -1) {
////            intakeServo.getServo().setPosition(intakeServoIn);
////        }
////        if (gamepad1.left_stick_button && !isStickPressed){
//        if (gamepad1.left_stick_button) {
//            speedFactor = 1.0;
////            isStickPressed = false;
////            if (speedFactor == 1.0){
////                speedFactor = 0.6;
////            } else {
////                speedFactor = 1.0;
////            }
//        } else {
//            speedFactor = 0.6;
////            isStickPressed = true;
//        }
//
//        if (gamepad1.right_stick_button) {
//            rotationFactor = 0.45;
//        } else {
//            rotationFactor = 0.6;
//        }
        if (gamepad2.right_stick_button && isStickClickReleased) {
            isStickClickReleased = false;
            if (phase == 0) {
                liftingThread1Object.start();
                phase++;
            } else if (phase == 1) {
                liftingThread2Object.start();
                phase++;
            } else if (phase == 2) {
                liftingThread2Object.start();
                phase = 0;
            }
        } else if (!gamepad2.right_stick_button){
            isStickClickReleased = true;
        }
//        if (gamepad2.y) {
//            intakeServo.run();
//        }
        if (gamepad1.left_bumper && !gamepad1.right_bumper) {
            machanumDrive.driveJoystick(gamepad1.left_stick_x, gamepad1.left_stick_y, 0.2, speedFactor, rotationFactor);
        } else if (gamepad1.right_bumper && !gamepad1.left_bumper) {
            machanumDrive.driveJoystick(gamepad1.left_stick_x, gamepad1.left_stick_y, -0.2, speedFactor, rotationFactor);
        } else {
            machanumDrive.driveJoystick(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, speedFactor, rotationFactor);
        }
        addedValue = 0;
        if (isSwang(gamepad2.right_stick_y) == 1 && assignedBricks < maxBricks && isStickReleased) {
            isStickReleased = false;
            addedValue = 1;
        } else if (isSwang(gamepad2.right_stick_y) == -1 && assignedBricks > 0 && isStickReleased) {
            isStickReleased = false;
            addedValue = -1;
        } else if (isSwang(gamepad2.right_stick_y) == 0) {
            addedValue = 0;
            isStickReleased = true;
        }
        telemetry.addData("pad2", isSwang(gamepad2.right_stick_y));
//
        assignedBricks += addedValue;
        telemetry.addData("assigned bricks", assignedBricks);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
//
//
//        if (gamepad2.b) {
//            capstoneServo.run(CapstoneServo.Direction.PUT);
//        }
//        if (gamepad2.a) {
//            capstoneServo.run(CapstoneServo.Direction.HOLD);
//        }


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
        if (stickValue < -0.7) {
            return 1;
        } else if (stickValue > 0.7) {
            return -1;
        } else {
            return 0;
        }
    }
}