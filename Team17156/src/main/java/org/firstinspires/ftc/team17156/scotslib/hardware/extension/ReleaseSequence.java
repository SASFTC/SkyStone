package org.firstinspires.ftc.team17156.scotslib.hardware.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team17156.scotslib.hardware.extension.IntakeServo;
import org.firstinspires.ftc.team17156.scotslib.hardware.extension.LiftingSystem;
import org.firstinspires.ftc.team17156.scotslib.hardware.extension.TurnAngle;

import static java.lang.Thread.sleep;

public class ReleaseSequence implements Runnable {
    private DcMotor operatingMotor;
    private double correction;
    private double avoidGrabberCollision;
    private double liftedHeight;
    private LiftingSystem liftingSys;
    public ReleaseSequence(DcMotor liftingMotor, double correction, double avoidGrabberCollision, double liftedHeight, LiftingSystem liftingSys){
        this.operatingMotor = liftingMotor;
        this.correction = correction;
        this.avoidGrabberCollision = avoidGrabberCollision;
        this.liftedHeight = liftedHeight;
        this.liftingSys = liftingSys;
    }

    @Override
    public void run(){
        this.operatingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.operatingMotor.setTargetPosition(-(int)correction);
        this.operatingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.operatingMotor.setPower(0.5);
        while (this.operatingMotor.isBusy()) {}
        this.operatingMotor.setPower(0);
        liftingSys.grabber(LiftingSystem.Grabber.RELEASE);
        this.operatingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.operatingMotor.setTargetPosition((int)avoidGrabberCollision);
        this.operatingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.operatingMotor.setPower(0.5);
        while (this.operatingMotor.isBusy()) {}
        this.operatingMotor.setPower(0);
        this.operatingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.operatingMotor.setTargetPosition(-(int)(liftedHeight + avoidGrabberCollision));
        this.operatingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.operatingMotor.setPower(0.5);
        while (this.operatingMotor.isBusy()) {}
        this.operatingMotor.setPower(0);


    }
}
