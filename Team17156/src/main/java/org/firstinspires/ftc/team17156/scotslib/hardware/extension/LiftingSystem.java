package org.firstinspires.ftc.team17156.scotslib.hardware.extension;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static com.qualcomm.robotcore.util.Range.clip;


public class LiftingSystem extends Extension {

    /* Fields */
    private DcMotor liftingMotor;
    private Servo wristServo, grabbingServo;

    private double maxLiftingSpeed;
    private double accel;

    public enum Direction { OUT, IN };
    public enum Grabber { GRAB, RELEASE };


    private final int LIFT_STEPS = 10; // TODO: Determine total steps to lift the arm, or use button to limit arm.
    private int currentLiftingStep = 0;



    /* Methods */
    public LiftingSystem(HardwareMap hardwareMap, String liftingMotor, String wristServo,
                         String grabbingServo, double maxLiftingSpeed, double accel) {

        // Get hardwareMap to access components.
        super(hardwareMap);

        // Get motors and servos.
        this.liftingMotor = super.get(DcMotor.class, liftingMotor);
        this.wristServo = super.get(Servo.class, wristServo);
        this.grabbingServo = super.get(Servo.class, grabbingServo);

        // Configure motors and servos.
        this.liftingMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.liftingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.liftingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.wristServo.scaleRange(0, 1);
        this.grabbingServo.scaleRange(0, 1);

        // Save metrics.
        this.maxLiftingSpeed = maxLiftingSpeed;
        this.accel = accel;
    }


    public void raise(double speed) {

        this.liftingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (this.liftingMotor.getCurrentPosition() <= LIFT_STEPS &&
                this.liftingMotor.getCurrentPosition() >= 0) {
            accelMotor(this.liftingMotor, speed);
        } else {
            this.stop();
        }
    }

    public void stop() {

        this.liftingMotor.setPower(0);
    }

    public void up() {

        this.liftingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (this.liftingMotor.getCurrentPosition() >= 0 &&
                this.liftingMotor.getCurrentPosition() <= LIFT_STEPS) {

            this.liftingMotor.setPower(this.maxLiftingSpeed);
        }

        this.stop();
    }

    public void down() {

        this.liftingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (this.liftingMotor.getCurrentPosition() >= 0 &&
                this.liftingMotor.getCurrentPosition() <= LIFT_STEPS) {

            this.liftingMotor.setPower(-this.maxLiftingSpeed);
        }

        this.stop();
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
