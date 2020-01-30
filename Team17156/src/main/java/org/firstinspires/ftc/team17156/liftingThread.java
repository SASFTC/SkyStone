package org.firstinspires.ftc.team17156;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.team17156.scotslib.hardware.extension.IntakeServo;
import org.firstinspires.ftc.team17156.scotslib.hardware.extension.LiftingSystem;

import static java.lang.Thread.sleep;

public class liftingThread implements Runnable {
    private LiftingSystem liftingSys;
    private IntakeServo intakeServo;
    private double bricks;
    public liftingThread(LiftingSystem liftingSys, IntakeServo intakeServo, double bricks){
        this.liftingSys = liftingSys;
        this.intakeServo = intakeServo;
        this.bricks = bricks;
    }

    @Override
    public void run() {
        this.intakeServo.run();
        this.liftingSys.getWristServo().setPosition(0.87);
        try{sleep(900);}
        catch (InterruptedException e){}
        this.liftingSys.getGrabbingServo().setPosition(0.31);
        try{sleep(200);}
        catch (InterruptedException e){}
        this.liftingSys.goBricks(1.0, bricks, true);
        this.liftingSys.getWristServo().setPosition(0.54);
        try{sleep(900);}
        catch (InterruptedException e){}
        this.liftingSys.goBricks(1.0, -0.01, true);
        this.liftingSys.getGrabbingServo().setPosition(0.7);
        try{sleep(500);}
        catch (InterruptedException e){}
        this.liftingSys.goBricks(1.0, 0.01, true);
        this.liftingSys.getWristServo().setPosition(0.87);
        this.liftingSys.goBricks(0.6, (int)(bricks/2), true);
        this.liftingSys.goBricks(0.3, (int)((bricks+1)/2), false);

    }
}
