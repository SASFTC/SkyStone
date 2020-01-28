package org.firstinspires.ftc.team17156.scotslib.hardware.drivetrain;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team17156.scotslib.hardware.extension.TurnAngle;

import static com.qualcomm.robotcore.util.Range.clip;


public class MecanumDrivetrain extends Drivetrain {

    /* Fields */
    private DcMotor motor_left_front, motor_right_front;
    private DcMotor motor_left_back, motor_right_back;

    private double accel;
    private double maxSpeed;

    // Keep track of some metrics.
    private double speed = 0;               //  -1 <=     speed     <= 1
    private double angle = 0;               // -pi <=     angle     <= pi
    private double rotation = 0;            //  -1 <=    rotation   <= 1
    private double wheelDiameter = 10.2;
    private int motorStepLF = 1680;
    private int motorStepLB = 1680;
    private int motorStepRF = 1680;
    private int motorStepRB = 1680;
    private HardwareMap hardwareMap;



    /* Methods */
    /**
     * Constructor for the Mecanum Drive class. Given an instance of all four motors,
     * it handles all the necessary math and logic to drive the mecanum Drivetrain.
     *
     * @param hardwareMap: The reference to the HardwareMap in the OpClass.
     * @param left_front:  The front-left motor.
     * @param right_front: The front-right motor.
     * @param left_back:   The back-left motor.
     * @param right_back:  The back-right motor.
     * @param accel:       The acceleration limit of the robot.
     * @param maxSpeed:    The maximum speed of the robot.
     */
    public MecanumDrivetrain(HardwareMap hardwareMap, String left_front, String right_front,
                             String left_back, String right_back, double accel, double maxSpeed,
                             boolean invertedDrive) {
        super(hardwareMap);
        // Get HardwareMap
        this.hardwareMap = hardwareMap;

        // Set the motors' instances.
        this.motor_left_front = super.get(DcMotor.class, left_front);
        this.motor_right_front = super.get(DcMotor.class, right_front);
        this.motor_left_back = super.get(DcMotor.class, left_back);
        this.motor_right_back = super.get(DcMotor.class, right_back);
//        super.get(DcMotor.class, "motor");

        // Set the motor orientation.
        if (!invertedDrive) {
            this.motor_left_front.setDirection(DcMotorSimple.Direction.FORWARD);
            this.motor_right_front.setDirection(DcMotorSimple.Direction.REVERSE);
            this.motor_left_back.setDirection(DcMotorSimple.Direction.FORWARD);
            this.motor_right_back.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            this.motor_left_front.setDirection(DcMotorSimple.Direction.REVERSE);
            this.motor_right_front.setDirection(DcMotorSimple.Direction.FORWARD);
            this.motor_left_back.setDirection(DcMotorSimple.Direction.FORWARD);
            this.motor_right_back.setDirection(DcMotorSimple.Direction.REVERSE);
        }


        // Turn on PID for the motors' velocity control.
//        this.motor_left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        this.motor_right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        this.motor_left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        this.motor_right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Set ZeroPowerBehavior to BRAKE, so that any forces acting on the robot will not cause movement.
        this.motor_left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor_right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor_left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor_right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set other params.
        this.accel = accel;
        this.maxSpeed = maxSpeed;
    }

    public MecanumDrivetrain(HardwareMap hardwareMap, String left_front, String right_front, String left_back, String right_back) {
        this(hardwareMap, left_front, right_front, left_back, right_back, 0.5, 1, false);
    }


    /**
     * The main driving method.
     * Sets the appropriate motor powers given the speed, angle, and rotation speed.
     *
     * @param speed:    The magnitude of the velocity [-1, 1].
     * @param angle:    The direction of the velocity [-pi, pi]. The angle is zero when going forward,
     *                  negative clockwise, and positive counterclockwise.
     * @param rotation: The rotational speed [-1, 1]. Clockwise is positive, and counterclockwise is negative.
     */
    public void drive(double speed, double angle, double rotation) {
        rotation *= 0.45;

        // Constrain speed to max speed.
        this.speed = clip(speed, -this.maxSpeed, this.maxSpeed);
        this.angle = angle;
        this.rotation = clip(-rotation, -this.maxSpeed, this.maxSpeed);

        // Calculate each motor's speed.
        double v1 = this.speed * Math.sin(this.angle + Math.PI / 4) - this.rotation;    // Left front motor.
        double v2 = this.speed * Math.cos(this.angle + Math.PI / 4) + this.rotation;    // Right front motor.
        double v3 = this.speed * Math.cos(this.angle + Math.PI / 4) - this.rotation;    // Left back motor.
        double v4 = this.speed * Math.sin(this.angle + Math.PI / 4) + this.rotation;    // Right back motor.

        // Apply the desired power to each motor.
        accelMotor(this.motor_left_front, clip(v1, -1, 1));
        accelMotor(this.motor_right_front, clip(v2, -1, 1));
        accelMotor(this.motor_left_back, clip(v3, -1, 1));
        accelMotor(this.motor_right_back, clip(v4, -1, 1));
    }
    public void drive(double speed, double angle, double rotation, double distance){
        double angleToRotate = distance/(Math.pow(this.wheelDiameter/2, 2)*Math.PI/360);
        // Constrain speed to max speed.
        TurnAngle angleTurnMotorLF = new TurnAngle(this.hardwareMap, this.motor_left_front, 1.0, 100.0, this.motorStepLF);
//        angleTurnMotorLF.turnTo();
        this.speed = clip(speed, -this.maxSpeed, this.maxSpeed);
        this.angle = angle;
        this.rotation = clip(-rotation, -this.maxSpeed, this.maxSpeed);

        // Calculate each motor's speed.
        double v1 = this.speed * Math.sin(this.angle + Math.PI / 4) - this.rotation;    // Left front motor.
        double v2 = this.speed * Math.cos(this.angle + Math.PI / 4) + this.rotation;    // Right front motor.
        double v3 = this.speed * Math.cos(this.angle + Math.PI / 4) - this.rotation;    // Left back motor.
        double v4 = this.speed * Math.sin(this.angle + Math.PI / 4) + this.rotation;    // Right back motor.

        // Apply the desired power to each motor.
        accelMotor(this.motor_left_front, clip(v1, -1, 1));
        accelMotor(this.motor_right_front, clip(v2, -1, 1));
        accelMotor(this.motor_left_back, clip(v3, -1, 1));
        accelMotor(this.motor_right_back, clip(v4, -1, 1));

        this.drive(0, 0, rotation);

    }


    /**
     * Adapts the (x,y) output of the controller joystick to the main drive method.
     * Even though the Logitech F310  Joysticks' Y axis are inverted, DO NOT pass negative value.
     * This function handles that automatically.
     *
     * @param x:        The x component of the joystick.
     * @param y:        The y component of the joystick.
     * @param rotation: The x component of the secondary joystick, controlling rotation.
     */
    public void driveJoystick(double x, double y, double rotation, double speedFactor, double rotationFactor) {

        // Calculate the necessary values.
        double speed = Math.hypot(x, y)*speedFactor;
        // NOTE: in our controller, the Y axis is inverted – 1 is back and -1 is forward, so we invert it.
        double angle = Math.atan2(-y, -x) - Math.PI / 2;   // Make angle = 0 when joystick is forward.
        rotation *= rotationFactor;
        // Call the main driving method.
        this.drive(speed, angle, rotation);
    }


    /**
     * Sets all the motor powers to zero.
     */
    public void stop() {

        // Set all motors to zero power.
        this.motor_left_front.setPower(0);
        this.motor_right_front.setPower(0);
        this.motor_left_back.setPower(0);
        this.motor_right_back.setPower(0);

    }

    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = clip(maxSpeed, -1, 1);
    }

    /* Helper Methods */

    /**
     * A trapezoidal acceleration control for the motors, to avoid abrupt accelerations/decelerations.
     *
     * @param motor:    The DcMotor to which the power is applied.
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
