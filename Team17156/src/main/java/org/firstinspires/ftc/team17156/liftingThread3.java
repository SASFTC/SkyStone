package org.firstinspires.ftc.team17156;

import org.firstinspires.ftc.team17156.scotslib.hardware.extension.IntakeServo;
import org.firstinspires.ftc.team17156.scotslib.hardware.extension.LiftingSystem;

import static java.lang.Thread.sleep;

public class liftingThread3 implements Runnable {
    private LiftingSystem liftingSys;
    private IntakeServo intakeServo;
    private double bricks;
    private Thread liftingThread;
    public liftingThread3(LiftingSystem liftingSys, IntakeServo intakeServo, double bricks){
        this.liftingSys = liftingSys;
        this.intakeServo = intakeServo;
        this.bricks = bricks;
    }


    public void start() {
        this.liftingThread = new Thread(this);
        this.liftingThread.run();
    }

    @Override
    public void run() {
        this.liftingSys.goBricks(1.0, 0, -1, 0, liftingSys, false, false, false);
        this.liftingSys.grabber(LiftingSystem.Grabber.RELEASE);
        try{sleep(500);}
        catch (InterruptedException e){}
        this.liftingSys.goBricks(1.0, 0, 0, 1, liftingSys, false, false, false);
        this.liftingSys.swingWrist(LiftingSystem.Direction.IN);
        try{sleep(500);}
        catch (InterruptedException e){}
        this.liftingSys.goBricks(0.4, (int)bricks, 0, -1, liftingSys, true, false, true);

    }
}