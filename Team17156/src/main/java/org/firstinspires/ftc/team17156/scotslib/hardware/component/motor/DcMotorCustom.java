package org.firstinspires.ftc.team17156.scotslib.hardware.component.motor;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DcMotorCustom {

    /* Fields */
    private DcMotor motor;
    private int currentPosition = 0;


    public enum Mode {
        BY_ANGLE, BY_REGULATED_POWER, BY_UNREGULATED_POWER
    }

    /* Methods */
    public DcMotorCustom(DcMotor motor, Mode mode, DcMotor.Direction direction, DcMotor.ZeroPowerBehavior brake) {

        // Get motor.
        this.motor = motor;

        // Configure motor.
        this.configMotor(mode, direction, brake);

    }


    public void configMotor(Mode mode, DcMotor.Direction direction, DcMotor.ZeroPowerBehavior brake) {

        this.motor.setDirection(direction);
        this.motor.setZeroPowerBehavior(brake);

        switch (mode) {

            case BY_ANGLE:
                this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;

            case BY_REGULATED_POWER:
                this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;

            case BY_UNREGULATED_POWER:
                this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
        }

    }


    public void runToPosition(double deg, double power) {

        int position = (int) (deg * (1680 / 360)) - this.currentPosition;
        this.configMotor(Mode.BY_ANGLE, DcMotor.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setTargetPosition(position);
        this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motor.setPower(Math.abs(power));

        while (this.motor.isBusy()) {

        }

        this.motor.setPower(0);

        this.currentPosition += this.motor.getCurrentPosition();
    }


    public void setPower(double power) {

        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor.setPower(power);

    }

    public void stop() {

        this.motor.setPower(0);
        this.currentPosition += this.motor.getCurrentPosition();
    }

    public int getCurrentPosition() {

        return this.motor.getCurrentPosition();
    }
}
