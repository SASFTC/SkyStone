package org.firstinspires.ftc.team17156.scotslib.hardware.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team17156.scotslib.hardware.extension.IntakeServo;
import org.firstinspires.ftc.team17156.scotslib.hardware.extension.LiftingSystem;
import org.firstinspires.ftc.team17156.scotslib.hardware.extension.TurnAngle;

import static java.lang.Thread.sleep;

public class motorThread implements Runnable {
    private DcMotorImplEx operatingMotor;
    private double step;
    private double motorStep;
    private double speed;
    public motorThread(DcMotorImplEx operatingMotor, double speed, double step, double motorStep){
        this.operatingMotor = operatingMotor;
        this.step = step;
        this.speed = speed;
        this.motorStep = motorStep;
    }

    @Override
    public void run(){
        this.operatingMotor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
        this.operatingMotor.setTargetPosition((int)step);
        this.operatingMotor.setMode(DcMotorImplEx.RunMode.RUN_TO_POSITION);
        this.operatingMotor.setPower(speed);
        while (this.operatingMotor.isBusy()) {}
        this.operatingMotor.setPower(0);
    }
}
