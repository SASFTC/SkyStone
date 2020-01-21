package org.firstinspires.ftc.team17156.scotslib.hardware.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static com.qualcomm.robotcore.util.Range.clip;

public class AutoGrabber {

    /* Fields */
    private HardwareMap hardwareMap;
    private Servo arm, hand;


    private double accel;
    private double maxSpeed;

    // Keep track of some metrics.
    private double speed = 0;               //  -1 <=     speed     <= 1



    /* Methods */

    /**
     * Constructor for the Mecanum Drive class. Given an instance of all four motors,
     * it handles all the necessary math and logic to drive the mecanum Drivetrain.
     *
     * @param hardwareMap: The reference to the HardwareMap in the OpClass.
     * @param intake1:     The left motor.
     * @param intake2:     The front-right motor.
     * @param maxSpeed:    The maximum speed of the robot.
     */
    public AutoGrabber(HardwareMap hardwareMap, String arm, String hand) {
        // Get HardwareMap
        this.hardwareMap = hardwareMap;

        // Set the motors' instances.
        this.arm = hardwareMap.servo.get(arm);
        this.hand = hardwareMap.servo.get(hand);

        // Set the motor orientation.


        // Turn on PID for the motors' velocity control.
//        this.motor_left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        this.motor_right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        this.motor_left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        this.motor_right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Set ZeroPowerBehavior to BRAKE, so that any forces acting on the robot will not cause movement.

        // Set other params.
    }


    /**
     * The main driving method.
     * Sets the appropriate motor powers given the speed, angle, and rotation speed.
     *
     * @param speed: The magnitude of the velocity [-1, 1].
     */
    public void rise() {

        // Apply the desired power to each motor.
//        accelMotor(this.intake1, clip(v1, -1, 1));
//        accelMotor(this.intake2, clip(v2, -1, 1));
        this.arm.setPosition(0.78);
        this.hand.setPosition(0.5);
    }


    /**
     * The main driving method.
     * Sets the appropriate motor powers given the speed, angle, and rotation speed.
     *
     * @param speed: The magnitude of the velocity [-1, 1].
     */
    public void lower() {

        // Constrain speed to max speed.

        // Calculate each motor's speed.

        // Apply the desired power to each motor.
//        accelMotor(this.intake1, clip(v1, -1, 1));
//        accelMotor(this.intake2, clip(v2, -1, 1));
        this.arm.setPosition(0.4);
        this.hand.setPosition(0.5);
    }
}
