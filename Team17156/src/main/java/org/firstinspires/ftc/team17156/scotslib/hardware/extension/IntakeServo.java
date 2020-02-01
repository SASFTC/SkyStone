package org.firstinspires.ftc.team17156.scotslib.hardware.extension;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Thread.sleep;

public class IntakeServo extends Extension {

    /* Fields */
    private Servo intakeServo;
    private double retractValue = 0.15;
    private double pushValue = 0.7;


    /* Methods */

    /**
     * Constructor for the Intake class.
     *
     * @param hardwareMap: The HardwareMap, to get access to the motors.
     * @param motor_left:  The left motor's name.
     * @param motor_right: The right motor's name.
     * @param speed:       The speed at which the intake should work [-1, 1].
     */
    public IntakeServo(HardwareMap hardwareMap, String intakeServoName) {

        // Get the HardwareMap.
        super(hardwareMap);

        // Set the motors' instances.
        intakeServo = super.get(Servo.class, intakeServoName);
        intakeServo.scaleRange(0, 1);
        this.run(Direction.RETRACT);
    }


    public enum Direction {
        RETRACT, PUSH
    }

    /**
     * Runs the block intake given a direction, and uses the previously-defined speed to intake it.
     *
     * @param d: the direction (IN, OUT, or STOP)
     */
    public void run(Direction d) {
        switch (d) {
            case RETRACT:
                intakeServo.setPosition(retractValue);
                break;
            case PUSH:
                intakeServo.setPosition(pushValue);
                break;
        }
    }

    public Servo getServo() {
        return this.intakeServo;
    }

    public void run() {
        this.run(Direction.PUSH);
        try {
            sleep(500);
        } catch (InterruptedException e) {
        }
        this.run(Direction.RETRACT);
    }


}
