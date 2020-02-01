package org.firstinspires.ftc.team17156.scotslib.hardware.extension;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team17156.scotslib.hardware.extension.IntakeServo;
import org.firstinspires.ftc.team17156.scotslib.hardware.extension.LiftingSystem;

import static java.lang.Thread.sleep;

public class autonomousThread implements Runnable {
    private DcMotorImplEx motor_left_front;
    private DcMotorImplEx motor_right_front;
    private DcMotorImplEx motor_left_back;
    private DcMotorImplEx motor_right_back;
    private double speed = 1.0;
    private double motorStep = 537.6;
    private double angleToTurn;
    private double finalSteps;
    private Thread liftingThread;
    public autonomousThread(DcMotorImplEx motor_left_front, DcMotorImplEx motor_right_front, DcMotorImplEx motor_left_back, DcMotorImplEx motor_right_back){
        this.motor_left_front = motor_left_front;
        this.motor_right_front = motor_right_front;
        this.motor_left_back = motor_left_back;
        this.motor_right_back = motor_right_back;
    }


    public void start() {
        this.liftingThread = new Thread(this);
        this.liftingThread.run();
    }

    @Override
    public void run() {
        angleToTurn = 720;
        finalSteps = angleToTurn*motorStep/360;
        this.motor_left_front.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
        this.motor_left_front.setTargetPosition((int)finalSteps);
        this.motor_left_front.setMode(DcMotorImplEx.RunMode.RUN_TO_POSITION);
        this.motor_left_front.setPower(speed);
        this.motor_right_front.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
        this.motor_right_front.setTargetPosition((int)finalSteps);
        this.motor_right_front.setMode(DcMotorImplEx.RunMode.RUN_TO_POSITION);
        this.motor_right_front.setPower(speed);
        this.motor_left_back.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
        this.motor_left_back.setTargetPosition((int)finalSteps);
        this.motor_left_back.setMode(DcMotorImplEx.RunMode.RUN_TO_POSITION);
        this.motor_left_back.setPower(speed);
        this.motor_right_back.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
        this.motor_right_back.setTargetPosition((int)finalSteps);
        this.motor_right_back.setMode(DcMotorImplEx.RunMode.RUN_TO_POSITION);
        this.motor_right_back.setPower(speed);
        while (this.motor_left_front.isBusy() && this.motor_right_front.isBusy() && this.motor_left_back.isBusy() && this.motor_right_back.isBusy()) {}
        this.motor_left_back.setPower(0);
        this.motor_right_front.setPower(0);
        this.motor_left_front.setPower(0);
        this.motor_right_back.setPower(0);
    }
}