package org.firstinspires.ftc.team17156.utils.control;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Mecanum_DriveTrain {

    /* Fields */
    private HardwareMap hardwareMap;
    private DcMotor motor_left_front, motor_right_front;
    private DcMotor motor_left_back, motor_right_back;

    private double accel;
    private double maxSpeed;

    // Keep track of some metrics.
    private double speed = 0;               //  -1 <=     speed     <= 1
    private double angle = 0;               // -pi <=     angle     <= pi
    private double rotationSpeed = 0;       //  -1 <= rotationSpeed <= 1



    /* Methods */
    /**
     * Constructor for the Mecanum Drive class. Given an instance of all four motors,
     * it handles all the necessary math and logic to drive the mecanum Drivetrain.
     * @param hardwareMap: The reference to the HardwareMap in the OpClass.
     * @param left_front: The front-left motor.
     * @param right_front: The front-right motor.
     * @param left_back: The back-left motor.
     * @param right_back: The back-right motor.
     * @param accel: The acceleration limit of the robot.
     * @param maxSpeed: The maximum speed of the robot.
     */
    public Mecanum_DriveTrain(HardwareMap hardwareMap, String left_front, String right_front, String left_back, String right_back, double accel, double maxSpeed) {
        // Get HardwareMap
        this.hardwareMap = hardwareMap;

        // Set the motors.
        this.motor_left_front = this.hardwareMap.get(DcMotor.class, left_front);
        this.motor_right_front = this.hardwareMap.get(DcMotor.class, right_front);
        this.motor_left_back = this.hardwareMap.get(DcMotor.class, left_back);
        this.motor_right_back = this.hardwareMap.get(DcMotor.class, right_back);

        // Set the motor orientation.
        this.motor_left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motor_right_front.setDirection(DcMotorSimple.Direction.FORWARD);
        this.motor_left_back.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motor_right_back.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set other params.
        this.accel = accel;
        this.maxSpeed = maxSpeed;
    }
    public Mecanum_DriveTrain(HardwareMap hardwareMap, String left_front, String right_front, String left_back, String right_back) {
        this(hardwareMap, left_front, right_front, left_back, right_back, 0.1, 1);
    }


    /**
     * The main driving method.
     * Sets the appropriate motor powers given the speed, angle, and rotation speed.
     * @param speed: The magnitude of the velocity [-1, 1].
     * @param angle: The direction of the velocity [0, 2pi].
     * @param rotationSpeed: The rotational speed [-1, 1].
     */
    public void drive(double speed, double angle, double rotationSpeed) {

        // Constrain speed to max speed.
        this.speed = this.constrain(speed, -this.maxSpeed, this.maxSpeed);
        this.angle = angle;
        this.rotationSpeed = this.constrain(rotationSpeed, -1, 1);

        // Calculate each motor's speed.
        double v1 = this.speed * Math.cos(this.angle - Math.PI/4) + this.rotationSpeed;    // Left front motor.
        double v2 = this.speed * Math.sin(this.angle - Math.PI/4) - this.rotationSpeed;    // Right front motor.
        double v3 = this.speed * Math.sin(this.angle - Math.PI/4) + this.rotationSpeed;    // Left back motor.
        double v4 = this.speed * Math.cos(this.angle - Math.PI/4) - this.rotationSpeed;    // Right back motor.

        // Apply the desired power to each motor.
        accelMotor(this.motor_left_front, v1);
        accelMotor(this.motor_right_front, v2);
        accelMotor(this.motor_left_back, v3);
        accelMotor(this.motor_right_back, v4);
    }


    /**
     * Adapts the (x,y) output of the controller joystick to the main drive method.
     * @param x: The NEGATIVE of the x component of the joystick.
     * @param y: The y component of the joystick.
     * @param rotation: The x component of the secondary joystick, controlling rotation.
     */
    public void driveJoystick(double x, double y, double rotation) {

        // Calculate the necessary values.
        double speed = Math.hypot(x, y);
        double angle = Math.atan2(y, x);

        // Call the main driving mehtod.
        this.drive(speed, angle , rotation);
    }


    /* Helper Methods */
    // TODO Test if this method really works, and figure out the optimal accel value.
    // Trapezoidal motor control.
    private void accelMotor(DcMotor motor, double setpoint) {

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

    /**
     * Constrains the value of a double to a specified range.
     * @param value: The value to be constrained.
     * @param min: The min of the range.
     * @param max: The max of the range.
     * @return The constrained value.
     */
    private double constrain(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }

}
