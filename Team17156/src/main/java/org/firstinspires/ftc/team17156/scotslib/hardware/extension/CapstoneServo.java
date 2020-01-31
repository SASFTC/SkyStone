package org.firstinspires.ftc.team17156.scotslib.hardware.extension;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Thread.sleep;

public class CapstoneServo extends Extension {

    /* Fields */
    private Servo capstoneServo;
    private double holdValue = 0.15;
    private double putValue = 0.77;


    /* Methods */

    /**
     * Constructor for the Intake class.
     *
     * @param hardwareMap: The HardwareMap, to get access to the motors.
     * @param motor_left:  The left motor's name.
     * @param motor_right: The right motor's name.
     * @param speed:       The speed at which the intake should work [-1, 1].
     */
    public CapstoneServo(HardwareMap hardwareMap, String capstoneServoName) {

        // Get the HardwareMap.
        super(hardwareMap);

        // Set the motors' instances.
        capstoneServo = super.get(Servo.class, capstoneServoName);
        this.run(Direction.HOLD);
        capstoneServo.scaleRange(0, 1);
//        this.run(Direction.HOLD);
    }


    public enum Direction {
        HOLD, PUT
    }

    /**
     * Runs the block intake given a direction, and uses the previously-defined speed to intake it.
     *
     * @param d: the direction (IN, OUT, or STOP)
     */
    public void run(Direction d) {
        switch (d) {
            case HOLD:
                capstoneServo.setPosition(holdValue);
                break;
            case PUT:
                capstoneServo.setPosition(putValue);
                break;
            default:
                capstoneServo.setPosition(holdValue);
                break;
        }
    }

    public Servo getServo() {
        return this.capstoneServo;
    }

    public void run() {
        this.run(Direction.PUT);
        try {sleep(500);
            this.run(Direction.HOLD);
        } catch (InterruptedException e) {}
    }


}
