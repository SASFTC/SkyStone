package org.firstinspires.ftc.team17156.scotslib.hardware.extension;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Thread.sleep;

public class FoundationHolder extends Extension {

    /* Fields */
    private Servo leftFoundation;
    private Servo rightFoundation;
    private double leftFoundationDefault = 0.15;
    private double rightFoundationDefault = 0.75;
    private double leftFoundationGrasp = 0.9;
    private double rightFoundationGrasp = 0.04;


    /* Methods */

    /**
     * Constructor for the Intake class.
     *
     * @param hardwareMap: The HardwareMap, to get access to the motors.
     * @param motor_left:  The left motor's name.
     * @param motor_right: The right motor's name.
     * @param speed:       The speed at which the intake should work [-1, 1].
     */
    public FoundationHolder(HardwareMap hardwareMap, String leftFoundationName, String rightFoundationName) {

        // Get the HardwareMap.
        super(hardwareMap);

        // Set the motors' instances.
        this.leftFoundation = super.get(Servo.class, leftFoundationName);
        leftFoundation.scaleRange(0, 1);
        this.rightFoundation = super.get(Servo.class, rightFoundationName);
        rightFoundation.scaleRange(0, 1);
        this.run(State.DEFAULT);
    }


    public enum State {
        DEFAULT, GRASP
    }

    /**
     * Runs the block intake given a direction, and uses the previously-defined speed to intake it.
     *
     * @param d: the direction (IN, OUT, or STOP)
     */
    public void run(State d) {
        switch (d) {
            case DEFAULT:
                this.leftFoundation.setPosition(leftFoundationDefault);
                this.rightFoundation.setPosition(rightFoundationDefault);
                break;
            case GRASP:
                this.leftFoundation.setPosition(leftFoundationGrasp);
                this.rightFoundation.setPosition(rightFoundationGrasp);
                break;
        }
    }

    public Servo getRightFoundation() {
        return this.rightFoundation;
    }
    public Servo getLeftFoundation() {
        return this.leftFoundation;
    }

}
