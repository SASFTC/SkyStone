package org.firstinspires.ftc.team17156.scotslib.hardware.component.motor;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;

public class DcMotorCustom {

    /* Fields */
    private DcMotorImplEx motor;

    private long homePosition;
    private DcMotorCustomMode mode;
    private Thread waitingThread;
    private boolean locked = false;
    private final double STEPS_PER_RADIAN = 1680 / (2*Math.PI);

    public enum DcMotorCustomMode {
        BY_ANGLE, BY_REGULATED_POWER, BY_UNREGULATED_POWER
    }


    /* Methods */
    public DcMotorCustom(DcMotorImplEx motor, DcMotorCustomMode mode, DcMotorImplEx.Direction direction, DcMotorImplEx.ZeroPowerBehavior brake) {

        // Get motor.
        this.motor = motor;

        // Configure motor.
        this.configureMotor(mode, direction, brake);

    }


    private void configureMotor(DcMotorCustomMode mode, DcMotorImplEx.Direction direction, DcMotorImplEx.ZeroPowerBehavior brake) {

        if (this.notLocked()) {
            // Set motor direction.
            this.motor.setDirection(direction);

            // Set motor Zero Power Behavior.
            this.motor.setZeroPowerBehavior(brake);

            // Save mode.
            this.mode = mode;

            // Configure mode.
            switch (mode) {

                case BY_UNREGULATED_POWER:

                    // Set mode.
                    this.motor.setMode(DcMotorImplEx.RunMode.RUN_WITHOUT_ENCODER);
                    break;

                case BY_ANGLE:

                    // Set mode.
                    this.homePosition += this.motor.getCurrentPosition(); // Always add position to home before resetting the encoder.
                    this.motor.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);
                    this.motor.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);
                    break;

                case BY_REGULATED_POWER:

                    // Set mode.
                    this.motor.setMode(DcMotorImplEx.RunMode.RUN_USING_ENCODER);
                    break;
            }
        }
    }


    public void setPower(double power) {

        if (this.notLocked()) {

            // Set power.
            this.motor.setPower(power);
        }
    }


    public boolean rotateAngle(double rad, double power) {

        int position = this.motor.getCurrentPosition() + (int) ((rad * this.STEPS_PER_RADIAN));

        return this.runToPosition(position, power);

    }


    public boolean runToPosition(int position, double power) {

        // If not in BY_ANGLE mode, this method should not do anything.
        if (!this.mode.equals(DcMotorCustomMode.BY_ANGLE) && this.notLocked()) {

            return false;

        } else {

            // Set position target.
            this.motor.setTargetPosition(position);

            // Set mode to run to position.
            this.motor.setMode(DcMotorImplEx.RunMode.RUN_TO_POSITION);

            // Apply power.
            this.motor.setPower(Math.abs(power));

            // Lock motor.
            this.locked = true;

            // Create Thread that waits for motor, so as to not block up the entire program.
            this.waitingThread = new Thread(new Runnable() {
                @Override
                public void run() {
                    waitToReachPosition();
                }
            });

            // Wait for motor on another Thread.
            this.waitingThread.start();

            try {
                this.waitingThread.join();
            } catch (Exception e) {
                e.printStackTrace();
            }

            this.locked = false;
            this.motor.setPower(0);


            return true;
        }

    }

    private void waitToReachPosition() {

        // Wait until the motor is done, then break it.
        while (this.motor.isBusy()) {

        }

        this.stop();
    }

    public void stop() {

        // Set power to 0.
        this.motor.setPower(0);
    }

    public boolean notLocked() {

        return !this.locked;
    }
}
