package org.firstinspires.ftc.team17156;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team17156.scotslib.hardware.extension.IntakeServo;
import org.firstinspires.ftc.team17156.scotslib.hardware.extension.LiftingSystem;

import static java.lang.Thread.sleep;

public class liftingThread1 implements Runnable {
    private LiftingSystem liftingSys;
    private IntakeServo intakeServo;
    private Thread liftingThread;
    public liftingThread1(LiftingSystem liftingSys, IntakeServo intakeServo){
        this.liftingSys = liftingSys;
        this.intakeServo = intakeServo;
    }


    public void start() {
        this.liftingThread = new Thread(this);
        this.liftingThread.run();
    }

    @Override
    public void run() {
        intakeServo.run();
        this.liftingSys.swingWrist(LiftingSystem.Direction.IN);
        try{sleep(400);}
        catch (InterruptedException e){}
        this.liftingSys.grabber(LiftingSystem.Grabber.GRAB);

    }
}