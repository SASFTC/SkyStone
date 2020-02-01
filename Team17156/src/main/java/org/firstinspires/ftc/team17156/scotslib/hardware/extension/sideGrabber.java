package org.firstinspires.ftc.team17156.scotslib.hardware.extension;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Thread.sleep;

public class sideGrabber extends Extension {

    /* Fields */
    private Servo wristServo;
    private Servo grabbingServo;
    private double wristServoDefault = 0.71;
    private double wristServoApproaching = 0.31;
    private double wristServoGrabbing = 0.28;
    private double wristServoTransporting = 0.61;
    private double sideGrabberDefault = 0.96;
    private double sideGrabberApproaching = 0.25;
    private double sideGrabberGrabbing = 0.93;


    /* Methods */

    /**
     * Constructor for the Intake class.
     *
     * @param hardwareMap: The HardwareMap, to get access to the motors.
     * @param motor_left:  The left motor's name.
     * @param motor_right: The right motor's name.
     * @param speed:       The speed at which the intake should work [-1, 1].
     */
    public sideGrabber(HardwareMap hardwareMap, String wristServoName, String grabbingServoName) {

        // Get the HardwareMap.
        super(hardwareMap);

        // Set the motors' instances.
        this.wristServo = super.get(Servo.class, wristServoName);
        wristServo.scaleRange(0, 1);
        this.grabbingServo = super.get(Servo.class, grabbingServoName);
        grabbingServo.scaleRange(0, 1);
        this.run(State.DEFAULT);
    }


    public enum State {
        DEFAULT, GRAB, TRANSPORT, APPROACH
    }

    /**
     * Runs the block intake given a direction, and uses the previously-defined speed to intake it.
     *
     * @param d: the direction (IN, OUT, or STOP)
     */
    public void run(State d) {
        switch (d) {
            case DEFAULT:
                this.grabbingServo.setPosition(sideGrabberDefault);
                this.wristServo.setPosition(wristServoDefault);
                break;
            case GRAB:
                this.grabbingServo.setPosition(sideGrabberGrabbing);
                this.wristServo.setPosition(wristServoGrabbing);
                break;
            case TRANSPORT:
                this.grabbingServo.setPosition(wristServoTransporting);
                break;
            case APPROACH:
                this.grabbingServo.setPosition(sideGrabberApproaching);
                this.wristServo.setPosition(wristServoApproaching);
                break;
        }
    }

    public Servo getWristServo() {
        return this.wristServo;
    }
    public Servo getGrabbingServo() {
        return this.grabbingServo;
    }

}
