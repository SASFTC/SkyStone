package org.firstinspires.ftc.team17156.scotslib.controller;

import static java.lang.System.currentTimeMillis;

public class PID {

    /* Fields */
    private double kP, kI, kD;
    private double errorSum = 0;
    private double lastError = 0;
    private double minValue, maxValue, setpoint;
    private long lastTime;
    private Direction direction;

    // The direction enum.
    public enum Direction {
        DIRECT(1), REVERSE(-1);

        int value;
        Direction(int i) {
            this.value = i;
        }
    }



    /* Methods */
    public PID(double kP, double kI, double kD, Direction direction, double minValue, double maxValue) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.direction = direction;
        this.minValue = minValue;
        this.maxValue = maxValue;
    }
    public PID() {
        this(1.0, 0.0, 0.0, Direction.DIRECT, -1, 1);
    }


    /**
     * Computes the correction for a given input and setpoint.
     * @param input: The input for the controller.
     * @param setpoint: The setpoint for the input.
     * @return The correction.
     */
    public double compute(double input, double setpoint) {
        this.setSetpoint(setpoint);
        return this.compute(input);
    }

    public double compute(double input) {

        // Calculate time elapsed.
        double dt = currentTimeMillis() - this.lastTime;


        // Calculate current error.
        double error = this.setpoint - input;


        // Calculate all terms.
        this.errorSum += error * dt;     // Integral term.
        double deriv = (error - lastError) / dt;


        // Save current time and lastError.
        this.lastTime = currentTimeMillis();
        this.lastError = error;

        // Calculate and return corrected value.
        double output = (this.kP * error) + (this.kI * this.errorSum) + (this.kD * deriv);
        output *= this.direction.value;

        return constrain(output);
    }

    public void setOutputLimits(double minValue, double maxValue) {
        this.minValue = minValue;
        this.maxValue = maxValue;
    }

    public void setTunings(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setControllerDirection(Direction direction) {
        this.direction = direction;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    private double constrain(double value) {
        return Math.min(Math.max(value, this.minValue), this.maxValue);
    }
}
