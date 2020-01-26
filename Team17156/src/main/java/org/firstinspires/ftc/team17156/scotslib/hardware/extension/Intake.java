package org.firstinspires.ftc.team17156.scotslib.hardware.extension;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static com.qualcomm.robotcore.util.Range.clip;

public class Intake extends Extension {

    /* Fields */
    private DcMotor motor_left, motor_right;

    private double speed;

    /**
     * Enum to store the different option for the intake mechanism, either in or out.
     */
    public enum Direction {
        IN, OUT, STOP
    }


    /* Methods */
    /**
     * Constructor for the Intake class.
     * @param hardwareMap: The HardwareMap, to get access to the motors.
     * @param motor_left: The left motor's name.
     * @param motor_right: The right motor's name.
     * @param speed: The speed at which the intake should work [-1, 1].
     */
    public Intake (HardwareMap hardwareMap, String motor_left, String motor_right, double speed) {

        // Get the HardwareMap.
        super(hardwareMap);

        // Set the motors' instances.
        this.motor_left = super.get(DcMotor.class, motor_left);
        this.motor_right = super.get(DcMotor.class, motor_right);

        // Set the motors' direction.
        this.motor_left.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motor_right.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set ZeroPowerBehavior to BRAKE, so that any forces acting on the intake will not cause movement.
        this.motor_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Other.
        this.speed = clip(speed, -1, 1);
    }

    /**
     * Runs the block intake given a direction, and uses the previously-defined speed to intake it.
     * @param d: the direction (IN, OUT, or STOP)
     */
    public void run(Direction d) {

        switch (d) {

            case IN:
                this.motor_left.setPower(this.speed);
                this.motor_right.setPower(this.speed);
                break;

            case OUT:
                this.motor_left.setPower(-this.speed);
                this.motor_right.setPower(-this.speed);
                break;

            default:
                this.motor_left.setPower(0);
                this.motor_right.setPower(0);
        }

    }

    /**
     * Runs the block intake based on a speed, ranging from -1 (out) to 1 (in).
     * @param speed: The speed to which it should spin.
     */
    public void run(double speed) {

        this.speed = clip(speed, -1, 1);

        this.motor_left.setPower(this.speed);
        this.motor_right.setPower(this.speed);
    }

    /**
     * Sets the maximum speed for the intake mechanism. Must be positive and [0, 1].
     * @param speed: The new speed.
     */
    public void setSpeed(double speed) {

        this.speed = clip(Math.abs(speed), 0, 1);
    }


}
