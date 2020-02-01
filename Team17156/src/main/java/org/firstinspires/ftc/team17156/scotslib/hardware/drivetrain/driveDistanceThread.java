//package org.firstinspires.ftc.team17156.scotslib.hardware.drivetrain;
//
//import com.qualcomm.robotcore.hardware.DcMotorImplEx;
//import com.qualcomm.robotcore.hardware.DcMotorImplEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.team17156.scotslib.hardware.extension.IntakeServo;
//import org.firstinspires.ftc.team17156.scotslib.hardware.extension.LiftingSystem;
//
//import static java.lang.Thread.sleep;
//
//public class driveDistanceThread implements Runnable {
//    private double speed;
//    private double angle;
//    private DcMotorImplEx motor;
//    private Thread liftingThread;
//    public driveDistanceThread(double speed, double angle, DcMotorImplEx motor){
//    }
//
//
//    public void start() {
//        this.liftingThread = new Thread(this);
//        this.liftingThread.run();
//    }
//
//    @Override
//    public void run() {
//        this.motor_left_front.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
//        this.motor_left_front.setTargetPosition((int)angle);
//        this.motor_left_front.setMode(DcMotorImplEx.RunMode.RUN_TO_POSITION);
//        this.motor_left_front.setPower(speed);
//        this.motor_left_front.setPower(0);
//        this.motor_right_front.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
//        this.motor_right_front.setTargetPosition((int)angle);
//        this.motor_right_front.setMode(DcMotorImplEx.RunMode.RUN_TO_POSITION);
//        this.motor_right_front.setPower(speed);
//        this.motor_right_front.setPower(0);
//        this.motor_left_back.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
//        this.motor_left_back.setTargetPosition((int)angle);
//        this.motor_left_back.setMode(DcMotorImplEx.RunMode.RUN_TO_POSITION);
//        this.motor_left_back.setPower(speed);
//        this.motor_left_back.setPower(0);
//        this.motor_right_back.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
//        this.motor_right_back.setTargetPosition((int)angle);
//        this.motor_right_back.setMode(DcMotorImplEx.RunMode.RUN_TO_POSITION);
//        this.motor_right_back.setPower(speed);
//        while (this.motor_left_front.isBusy()) {}
//        while (this.motor_right_front.isBusy()) {}
//        while (this.motor_left_back.isBusy()) {}
//        while (this.motor_right_back.isBusy()) {}
//        this.motor_right_back.setPower(0);
//    }
//}