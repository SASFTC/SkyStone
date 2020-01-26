package org.firstinspires.ftc.team17156.scotslib.hardware.extension;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team17156.scotslib.hardware.component.motor.DcMotorCustom;


public class LiftingSystem extends Extension {

    /* Fields */
    private DcMotorCustom liftingMotor;
    private Servo wristServo, grabbingServo;

    private double maxLiftingSpeed = 1;

    public enum Direction { OUT, IN };
    public enum Grabber { GRAB, RELEASE };


    private final int LIFT_STEPS = 550 * 1680 / 360;
    private int currentLiftingStep = 0;



    /* Methods */
    public LiftingSystem(HardwareMap hardwareMap, String liftingMotor, String wristServo,
                         String grabbingServo, double maxLiftingSpeed) {

        // Get hardwareMap to access components.
        super(hardwareMap);

        // Get motors and servos.
        this.liftingMotor = new DcMotorCustom(super.get(DcMotor.class, liftingMotor),
                DcMotorCustom.Mode.BY_ANGLE, DcMotorSimple.Direction.FORWARD,
                DcMotor.ZeroPowerBehavior.BRAKE);
        this.wristServo = super.get(Servo.class, wristServo);
        this.grabbingServo = super.get(Servo.class, grabbingServo);

        // Configure motors and servos.
        this.wristServo.scaleRange(0, 1);
        this.grabbingServo.scaleRange(0, 1);

        // Save metrics.
        this.maxLiftingSpeed = maxLiftingSpeed;
    }


    public void raise(double power) {

        if (power > 0 && this.liftingMotor.getCurrentPosition() <= this.LIFT_STEPS) {
            this.liftingMotor.setPower(power);
        } else  if (power < 0 && this.liftingMotor.getCurrentPosition() > 0) {
            this.liftingMotor.setPower(power);
        } else {
            this.liftingMotor.stop();
        }
    }

    public void stop() {

        this.liftingMotor.stop();
    }

    public void up() {

        this.liftingMotor.runToPosition(this.LIFT_STEPS, this.maxLiftingSpeed);
    }

    public void down() {

        this.liftingMotor.runToPosition(0, 0.6);
    }

    public void swingWrist(Direction d) {

        switch (d) {

            case OUT:
                // Activate servo on the wrist to swing the arm outward.
                this.wristServo.setPosition(0);
                break;

            case IN:
                // Activate servo on the wrist to swing the arm inward.
                this.wristServo.setPosition(0.45);
                break;

        }
    }

    public void grabber(Grabber g) {

        switch (g) {

            case GRAB:
                // Activate servo on the wrist to swing the arm outward.
                this.grabbingServo.setPosition(0.2);
                break;

            case RELEASE:
                // Activate servo on the wrist to swing the arm inward.
                this.grabbingServo.setPosition(1);
                break;

        }
    }
}
