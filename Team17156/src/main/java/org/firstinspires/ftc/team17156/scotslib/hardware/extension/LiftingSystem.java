package org.firstinspires.ftc.team17156.scotslib.hardware.extension;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.qualcomm.robotcore.util.Range.clip;


public class LiftingSystem extends Extension {

    /* Fields */
    private DcMotor liftingMotor;
    private Servo wristServo, grabbingServo;

    private double maxLiftingSpeed;
    private double accel;

    public enum Direction {OUT, IN}

    ;

    public enum Grabber {GRAB, RELEASE}

    ;

    // TODO: Determine total steps to lift the arm, or use button to limit arm.
    private final int brickTall = 360;
    private final int correction = 0;
    private double liftingMotorStep;
    private HardwareMap hardwareMap;


    /* Methods */
    public LiftingSystem(HardwareMap hardwareMap, String liftingMotor, String wristServo,
                         String grabbingServo, double maxLiftingSpeed, double accel, double liftingMotorStep) {

        // Get hardwareMap to access components.
        super(hardwareMap);
        this.hardwareMap = hardwareMap;
        this.liftingMotorStep = liftingMotorStep;
        // Get motors and servos.
        this.liftingMotor = super.get(DcMotor.class, liftingMotor);
        this.wristServo = super.get(Servo.class, wristServo);
        this.grabbingServo = super.get(Servo.class, grabbingServo);

        // Configure motors and servos.
        this.liftingMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.liftingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.liftingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.wristServo.scaleRange(0, 1);
        this.grabbingServo.scaleRange(0, 1);

        // Save metrics.
        this.maxLiftingSpeed = maxLiftingSpeed;
        this.accel = accel;
    }
    public LiftingSystem(HardwareMap hardwareMap, String liftingMotor, double maxLiftingSpeed, double accel, double liftingMotorStep) {

        // Get hardwareMap to access components.
        super(hardwareMap);
        this.hardwareMap = hardwareMap;
        this.liftingMotorStep = liftingMotorStep;
        // Get motors and servos.
        this.liftingMotor = super.get(DcMotor.class, liftingMotor);

        // Configure motors and servos.
        this.liftingMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.liftingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.liftingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Save metrics.
        this.maxLiftingSpeed = maxLiftingSpeed;
        this.accel = accel;
    }

    public void goBricks(double speed, int bricks, Telemetry telemetry) {
        TurnAngle raisingMotor = new TurnAngle(hardwareMap, this.liftingMotor, 1.0, 100, this.liftingMotorStep);
        raisingMotor.turnTo(speed, bricks*brickTall+correction, telemetry);
    }

    public void stop() {

        this.liftingMotor.setPower(0);
    }

    public void swingWrist(Direction d) {

        switch (d) {

            case OUT:
                // Activate servo on the wrist to swing the arm outward.
                this.wristServo.setPosition(1); // TODO: Determine angle value.
                break;

            case IN:
                // Activate servo on the wrist to swing the arm inward.
                this.wristServo.setPosition(0); // TODO: Determine angle value.
                break;

        }
    }
    public double getMotorPosition(){
        return liftingMotor.getCurrentPosition();
    }

    public void grabber(Grabber g) {

        switch (g) {

            case GRAB:
                // Activate servo on the wrist to swing the arm outward.
                this.wristServo.setPosition(0.5); // TODO: Determine angle value.
                break;

            case RELEASE:
                // Activate servo on the wrist to swing the arm inward.
                this.wristServo.setPosition(0); // TODO: Determine angle value.
                break;

        }
    }


    /**
     * A trapezoidal acceleration control for the motors, to avoid abrupt accelerations/decelerations.
     *
     * @param motor:    The DcMotor to which the power is applied.
     * @param setSpeed: The final velocity [-1, 1].
     */
    private void accelMotor(DcMotor motor, double setSpeed) {

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
}
