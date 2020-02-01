package org.firstinspires.ftc.team17156;

import org.firstinspires.ftc.team17156.scotslib.hardware.extension.IntakeServo;
import org.firstinspires.ftc.team17156.scotslib.hardware.extension.LiftingSystem;

import static java.lang.Thread.sleep;

public class liftingThread2 implements Runnable {
    private LiftingSystem liftingSys;
    private IntakeServo intakeServo;
    private double bricks;
    private Thread liftingThread;
    public liftingThread2(LiftingSystem liftingSys, IntakeServo intakeServo, double bricks){
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
        this.liftingSys.goBricks(1.0, bricks, 1, 0, liftingSys, true, true, false);
        if (bricks > 2 || bricks < 0)
            this.liftingSys.swingWrist(LiftingSystem.Direction.OUT);
    }
}