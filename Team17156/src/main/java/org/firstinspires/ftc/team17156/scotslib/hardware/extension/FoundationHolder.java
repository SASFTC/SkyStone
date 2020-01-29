//package org.firstinspires.ftc.team17156.scotslib.hardware.extension;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//public class FoundationHolder extends Extension {
//
//    /* Fields */
//    private Servo left_servo, right_servo;
//
//    private final double GRAB = 0; // TODO: Determine servo value;
//    private final double RELEASE = 180; // TODO: Determine servo value;
//
//
//    /* Methods */
//    public FoundationHolder(HardwareMap hardwareMap, String left_servo, String right_servo) {
//
//        // Get HardwareMap.
//        super(hardwareMap);
//
//        // Get Servos.
//        this.left_servo = super.get(Servo.class, left_servo);
//        this.right_servo = super.get(Servo.class, right_servo);
//
//        // Set servos. Since they are in mirrored positions, invert directions.
//        this.left_servo.setDirection(Servo.Direction.FORWARD);
//        this.left_servo.scaleRange(0, 1);
//        this.left_servo.setDirection(Servo.Direction.REVERSE);
//        this.right_servo.scaleRange(0, 1);
//
//        // Reset Servos to original position.
//        this.release();
//
//    }
//
//
//    public void grab() {
//
//        this.left_servo.setPosition(this.GRAB);
//        this.right_servo.setPosition(this.GRAB);
//    }
//
//    public void release() {
//
//        this.left_servo.setPosition(this.RELEASE);
//        this.right_servo.setPosition(this.RELEASE);
//    }
//}
