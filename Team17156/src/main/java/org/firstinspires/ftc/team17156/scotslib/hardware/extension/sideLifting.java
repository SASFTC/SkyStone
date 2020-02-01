package org.firstinspires.ftc.team17156.scotslib.hardware.extension;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Thread.sleep;

public class sideLifting extends Extension {

    /* Fields */
    private double groundToFirst = 0.15;
    private double firstToSecond = 0.7;
    private int currentFloor = 0;
    private TurnAngle liftingServo;
    private double speed;


    /* Methods */

    /**
     * Constructor for the Intake class.
     *
     * @param hardwareMap: The HardwareMap, to get access to the motors.
     * @param motor_left:  The left motor's name.
     * @param motor_right: The right motor's name.
     * @param speed:       The speed at which the intake should work [-1, 1].
     */
    public sideLifting(HardwareMap hardwareMap, String liftingServoName, int motorStep, double speed) {

        // Get the HardwareMap.
        super(hardwareMap);
        this.speed = speed;
        // Set the motors' instances.
        liftingServo = new TurnAngle(hardwareMap, liftingServoName, 1.0, 100, motorStep);
    }


    public enum Direction {
        UP, DOWN
    }

    /**
     * Runs the block intake given a direction, and uses the previously-defined speed to intake it.
     *
     * @param d: the direction (IN, OUT, or STOP)
     */
    public void run(Direction d) {
        double liftValue = 0;
        switch (d) {
            case UP:
                if (currentFloor == 0)
                    liftValue = groundToFirst;
                else if (currentFloor == 1)
                    liftValue = firstToSecond;
            case DOWN:
                if (currentFloor == 1)
                    liftValue = -groundToFirst;
                else if (currentFloor == 0)
                    liftValue = -firstToSecond;
        }
        liftingServo.turn(speed, liftValue, false);
    }

    public TurnAngle getMotor() {
        return this.liftingServo;
    }


}
