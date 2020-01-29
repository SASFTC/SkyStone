package org.firstinspires.ftc.team17156.scotslib.hardware.component.motor;

import org.firstinspires.ftc.team17156.scotslib.configuration.SAConstants;

import static java.lang.Thread.sleep;

public class DcMotorControllerSA implements Runnable {

    /* Fields */
    private static final int MAX_NUMBER_MOTORS = 8;
    private static final int REFRESH_RATE = SAConstants.MOTOR_CONTROLLER_REFRESH_RATE; // In Hz.
    private static final int REFRESH_PERIOD = 1000 / REFRESH_RATE; // In ms.

    private DcMotorSA[] motors = new DcMotorSA[MAX_NUMBER_MOTORS];
    private boolean isRunning = false;
    private int motorCount = 0;
    private Thread motorControlThread;



    /* Methods */
    public DcMotorControllerSA() {
    }

    public void addMotor(DcMotorSA motor) {

        // Add motor to array.
        this.motors[motorCount] = motor;
        this.motorCount++;
    }

    @Override
    public void run() {

        // While the controller is running.
        while (this.isRunning) {

            // Iterate through every motor and call update().
            for (DcMotorSA motor: this.motors) {
                if (motor!=null)
                    motor.update();
            }

            // Sleep to refresh rate.
            try {
                sleep(this.REFRESH_PERIOD);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void start() {

        // If not already running.
        if (!this.isRunning) {

            // Create thread and set flag to run.
            this.isRunning = true;
            this.motorControlThread = new Thread(this);
            this.motorControlThread.start();
        }
    }

    public void stop() {

        // Set flag.
        this.isRunning = false;

        // Stop all motors.
        for (DcMotorSA motor: this.motors) {
            if (motor!=null )
                motor.hardStop();
        }
    }
}
