package org.firstinspires.ftc.team17156.scotslib.hardware.extension;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.team17156.scotslib.hardware.drivetrain.ReleaseSequence;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Thread.sleep;


public class LiftingSystem extends Extension {

    /* Fields */
    private DcMotorImplEx liftingMotor;
    private Servo wristServo, grabbingServo;

    private double maxLiftingSpeed;
    private double accel;

    public enum Direction {OUT, IN}

    ;

    public enum Grabber {GRAB, RELEASE}

    ;

    // TODO: Determine total steps to lift the arm, or use button to limit arm.
//    private final int brickTall = 170;
    private final int correction = 170;
    private final int brick0tall = 269;
    private final int brick1tall = 567;
    private final int brick2tall = 893;
    private final int brick3tall = 1200;
    private final int brick4tall = 1504;
    private final int brick5tall = 1790;
    private final int brick6tall = 1990;
    private final int avoidStructuralCollision = 650;
    private double startingHeight = -8;
    private final int avoidGrabberCollision = 330;
    private double liftingMotorStep;
    private HardwareMap hardwareMap;
    private double liftedHeight = 0;
    TurnAngle raisingMotor;


    /* Methods */
    public LiftingSystem(HardwareMap hardwareMap, String liftingMotor, String wristServo,
                         String grabbingServo, double maxLiftingSpeed, double accel, double liftingMotorStep) {

        // Get hardwareMap to access components.
        super(hardwareMap);
        this.hardwareMap = hardwareMap;
        this.liftingMotorStep = liftingMotorStep;
        // Get motors and servos.
        this.liftingMotor = super.get(DcMotorImplEx.class, liftingMotor);
        this.wristServo = super.get(Servo.class, wristServo);
        this.grabbingServo = super.get(Servo.class, grabbingServo);

        // Configure motors and servos.
        this.liftingMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.liftingMotor.setZeroPowerBehavior(DcMotorImplEx.ZeroPowerBehavior.BRAKE);
        this.liftingMotor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);

        this.wristServo.scaleRange(0, 1);
        this.grabbingServo.scaleRange(0, 1);
        raisingMotor = new TurnAngle(hardwareMap, this.liftingMotor, 1.0, 100, this.liftingMotorStep);


        // Save metrics.
        this.maxLiftingSpeed = maxLiftingSpeed;
        this.accel = accel;
        this.swingWrist(Direction.IN);
        this.grabber(Grabber.RELEASE);
    }

    public void goBricks(double speed, double bricks, int correctionFactor, int avoidGrabberCollisionFactor, LiftingSystem liftingSys, boolean brickCounts, boolean crossBarrier, boolean reverseDirection) {
        double finalAngle = 0;
        if (brickCounts) {
            switch (Math.abs((int) bricks)) {
                case 0:
                    finalAngle = brick0tall;
                    break;
                case 1:
                    finalAngle = brick1tall;
                    break;
                case 2:
                    finalAngle = brick2tall;
                    break;
                case 3:
                    finalAngle = brick3tall;
                    break;
                case 4:
                    finalAngle = brick4tall;
                    break;
                case 5:
                    finalAngle = brick5tall;
                    break;
                case 6:
                    finalAngle = brick6tall;
                    break;
            }
        }
        if (reverseDirection) {
            finalAngle = -finalAngle;
        }
        finalAngle += correction * correctionFactor;
        finalAngle += avoidGrabberCollision * avoidGrabberCollisionFactor;
        if (bricks < 2 && bricks >= 0 && crossBarrier) {
            raisingMotor.turn(speed, avoidStructuralCollision, true);
            liftingSys.swingWrist(Direction.OUT);
            try {
                sleep(600);
            } catch (InterruptedException e) {
            }
            finalAngle -= avoidStructuralCollision;
        }
        raisingMotor.turn(speed, finalAngle, true);
    }

    public void dropBrick() {
//        Thread release = new Thread(new ReleaseSequence(liftingMotor, correction, avoidGrabberCollision, liftedHeight, this));
//        release.start();
    }

    public boolean goBricksIsComplete() {
        return raisingMotor.isTurnComplete();
    }

    public void stop() {

        this.liftingMotor.setPower(0);
    }

    public void swingWrist(Direction d) {

        switch (d) {

            case OUT:
                // Activate servo on the wrist to swing the arm outward.
                this.wristServo.setPosition(0.02); // TODO: Determine angle value.
                break;

            case IN:
                // Activate servo on the wrist to swing the arm inward.
                this.wristServo.setPosition(0.5); // TODO: Determine angle value.
                break;

        }
    }

    public double getMotorPosition() {
        return liftingMotor.getCurrentPosition();
    }

    public void grabber(Grabber g) {

        switch (g) {

            case GRAB:
                // Activate servo on the wrist to swing the arm outward.
                this.grabbingServo.setPosition(0.31); // TODO: Determine angle value.
                break;

            case RELEASE:
                // Activate servo on the wrist to swing the arm inward.
                this.grabbingServo.setPosition(0.7); // TODO: Determine angle value.
                break;

        }
    }


    /**
     * A trapezoidal acceleration control for the motors, to avoid abrupt accelerations/decelerations.
     *
     * @param motor:    The DcMotorImplEx to which the power is applied.
     * @param setSpeed: The final velocity [-1, 1].
     */
    private void accelMotor(DcMotorImplEx motor, double setSpeed) {

        double newSpeed = 0;

        // If current power is bigger than what the acceleration limit allows (either way).
        if (this.accel < Math.abs(motor.getPower() - setSpeed)) {
            // If current power is lower than what it should be.
            if (motor.getPower() - setSpeed < 0)
                newSpeed = motor.getPower() + this.accel;
                // If current power is higher than what it should be.
            else
                newSpeed = motor.getPower() - this.accel;
        }
        // If current power is within acceleration limits.
        else {
            newSpeed = setSpeed;
        }

        newSpeed = clip(newSpeed, -this.maxLiftingSpeed, this.maxLiftingSpeed);
        motor.setPower(newSpeed);
    }

    public DcMotorImplEx getLiftingMotor() {
        return liftingMotor;
    }

    public void retractWire(){
        this.liftingMotor.setPower(0.1);
        try {
            sleep(5000);
        } catch (InterruptedException e) {
        }
        this.liftingMotor.setPower(0);
    }

    public Servo getWristServo() {
        return wristServo;
    }

    public Servo getGrabbingServo() {
        return grabbingServo;
    }
}
