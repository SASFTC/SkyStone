package org.firstinspires.ftc.team17156.scotslib.hardware.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static com.qualcomm.robotcore.util.Range.clip;

public class Intake {

    /* Fields */
    private HardwareMap hardwareMap;
    private DcMotor intake1, intake2;


    private double accel;
    private double maxSpeed;

    // Keep track of some metrics.
    private double speed = 0;               //  -1 <=     speed     <= 1



    /* Methods */
    /**
     * Constructor for the Mecanum Drive class. Given an instance of all four motors,
     * it handles all the necessary math and logic to drive the mecanum Drivetrain.
     * @param hardwareMap: The reference to the HardwareMap in the OpClass.
     * @param intake1: The left motor.
     * @param intake2: The front-right motor.
     * @param maxSpeed: The maximum speed of the robot.
     */
    public Intake(HardwareMap hardwareMap, String intake1, String intake2,
                             double maxSpeed) {
        // Get HardwareMap
        this.hardwareMap = hardwareMap;

        // Set the motors' instances.
        this.intake1 = this.hardwareMap.get(DcMotor.class, intake1);
        this.intake2 = this.hardwareMap.get(DcMotor.class, intake2);

        // Set the motor orientation.
        this.intake1.setDirection(DcMotorSimple.Direction.FORWARD);
        this.intake2.setDirection(DcMotorSimple.Direction.REVERSE);


        // Turn on PID for the motors' velocity control.
//        this.motor_left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        this.motor_right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        this.motor_left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        this.motor_right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Set ZeroPowerBehavior to BRAKE, so that any forces acting on the robot will not cause movement.
        this.intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set other params.
        this.maxSpeed = maxSpeed;
    }


    /**
     * The main driving method.
     * Sets the appropriate motor powers given the speed, angle, and rotation speed.
     * @param speed: The magnitude of the velocity [-1, 1].
     */
    public void run(double speed) {

        // Constrain speed to max speed.
        this.speed = clip(speed, -this.maxSpeed, this.maxSpeed);

        // Calculate each motor's speed.
        double v1 = this.speed + Math.sin(Math.PI/4);    // Intake1 motor.
        double v2 = this.speed + Math.sin(Math.PI/4);    // Intake2 motor.

        // Apply the desired power to each motor.
//        accelMotor(this.intake1, clip(v1, -1, 1));
//        accelMotor(this.intake2, clip(v2, -1, 1));
        this.intake1.setPower(v1);
        this.intake2.setPower(v2);
    }

    /**
     * Sets all the motor powers to zero.
     */
    public void stop() {

        // Set all motors to zero power.
        this.intake1.setPower(0);
        this.intake2.setPower(0);

    }

    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = clip(maxSpeed, -1, 1);
    }

    /* Helper Methods */
    /**
     * A trapezoidal acceleration control for the motors, to avoid abrupt accelerations/decelerations.
     * @param motor: The DcMotor to which the power is applied.
     * @param setpoint: The final velocity [-1, 1].
     */
    private void accelMotor(DcMotor motor, double setpoint) {

        // NOTE: getPower() returns the power in the interval [0, 1].
        // Therefore, we should use the abs value of setpoint [-1, 1].
//        setpoint = Math.abs(setpoint);

        // If current power is bigger than what the acceleration limit allows (either way).
        if (this.accel < Math.abs(motor.getPower() - setpoint)) {
            // If current power is lower than what it should be.
            if (motor.getPower() - setpoint < 0)
                motor.setPower(motor.getPower() + this.accel);
                // If current power is higher than what it should be.
            else
                motor.setPower(motor.getPower() - this.accel);
        }
        // If current power is within acceleration limits.
        else {
            motor.setPower(setpoint);
        }
    }
}
