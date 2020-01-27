package org.firstinspires.ftc.team17156.scotslib.hardware.component.motor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team17156.scotslib.configuration.SAConstants;

import static com.qualcomm.robotcore.util.Range.clip;

public class DcMotorSA {

    /* Fields */
    private DcMotorImplEx motor;

    private static final double MAX_POWER = 1;
    private static final double MIN_POWER = -1;

    private double acceleration, deceleration;
    private double currentPower, targetPower;
    private double minimumPower = SAConstants.DC_MOTOR_MINIMUM_POWER;
    private int currentPosition;


    // Enums.
    public enum Mode {BY_ANGLE, BY_VELOCITY}



    /* Methods */

    // FOR ACCEL:
    public double getCurrentPower() {
        return this.motor.getPower();
    }

    public synchronized double getTargetPower() {
        return this.targetPower;
    }

    public synchronized double getVelocity(AngleUnit unit) {
        return this.motor.getVelocity(unit);
    }

    public synchronized int getTargetPositionTolerance() {
        return this.getTargetPositionTolerance();
    }

    public synchronized boolean enabled() {
        return this.motor.isMotorEnabled();
    }

    public synchronized void enable(boolean enable) {

        if (enable) {
            this.motor.setMotorEnable();
        } else {
            this.motor.setMotorDisable();
        }
    }

    public synchronized void setTargetPositionTolerance(int tolerance) {
        this.motor.setTargetPositionTolerance(tolerance);
    }

    public synchronized void setVelocity(double velocity, AngleUnit unit) {
        this.motor.setVelocity(velocity, unit);
    }

    public synchronized void setPIDCoefficients(Mode mode, double p, double i, double d) {

        switch (mode) {
            case BY_ANGLE:
                this.motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(p, i, d, 0));
                break;

            case BY_VELOCITY:
                this.motor.setVelocityPIDFCoefficients(p, i, d, 0); // F is a bias added to the output.
                break;
        }
    }

    public synchronized boolean isBusy() {
        return this.motor.isBusy();
    }

    public synchronized void setDirection(DcMotor.Direction direction) {
        this.motor.setDirection(direction);
    }

    public synchronized void setMode(Mode mode) {
        // TODO: Set mode appropriately.
    }

    public synchronized void setAcceleration(double acceleration, double deceleration) {

        // Update values.
        this.acceleration = acceleration;
        this.deceleration = deceleration;
    }

    public synchronized void setMinimumPower(double minimumPower) {

        // Update value.
        this.minimumPower = minimumPower;
    }

    public synchronized void setTargetPower(double power) {

        // Update value.
        this.targetPower = power;
    }

    public synchronized void setPower(double power) {

        // Apply power to motor directly.
        this.motor.setPower(power);
    }

    public void hardStop() {

        // Break motor.
        this.motor.setPower(0);
    }

    public synchronized void update() { // THIS SHOULD TAKE CARE OF EVERYTHING THAT REQUIRES A LOOP

        // TODO: PID for the stepper mode?
    }

    private synchronized void updatePower() {

        // If going forward.
        if (this.currentPower > 0) {

            // Accelerating.
            if (this.currentPower < this.targetPower) {
                this.currentPower += this.acceleration;
                this.currentPower = Math.min(this.currentPower, this.targetPower);
            }

            // Decelerating.
            else if (this.currentPower > this.targetPower) {
                this.currentPower -= this.deceleration;
                this.currentPower = Math.max(this.currentPower, 0);
            }
        }

        // If going backward.
        else if (this.currentPower < 0) {

            // Decelerating.
            if (this.currentPower < this.targetPower) {
                this.currentPower += this.deceleration;
                this.currentPower = Math.min(this.currentPower, 0);
            }

            // Accelerating.
            else if (this.currentPower > this.targetPower) {
                this.currentPower -= this.acceleration;
                this.currentPower = Math.max(this.currentPower, this.targetPower);
            }
        }

        // If stopped.
        else {

            // If intended to move forward.
            if (this.targetPower > 0) {
                this.currentPower += Math.max(this.minimumPower, this.acceleration);
            }

            // If intended to go backward.
            else if (this.targetPower > 0) {
                this.currentPower -= Math.max(this.minimumPower, this.acceleration);
            }
        }

        // Finally, after determining the new currentPower, clip it and apply it.
        this.currentPower = clip(this.currentPosition, this.MIN_POWER, this.MAX_POWER);
        this.setPower(this.currentPower);

    }

    private synchronized void updateStepper() {

        // If motor is not busy anymore, break it.
        if (!this.motor.isBusy()) {
            this.hardStop();
        }
    }

    // FOR STEPPER:
    public void runToPosition(int position) {
        // TODO
    }

    public void rotateAngle(double angle) {
        // TODO
    }

    public synchronized int getAbsolutePosition() {
        // TODO
        return 0;
    }



}
