package org.firstinspires.ftc.team17156.scotslib.hardware.extension;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.qualcomm.robotcore.util.Range.clip;


public class TurnAngle extends Extension {

    /* Fields */
    private DcMotor motor;

    private double maxSpeed;
    private double accel;
    private double motorStep;


    // TODO: Determine total steps to lift the arm, or use button to limit arm.
    private final int LIFT_STEPS = 10;


    /* Methods */
    public TurnAngle(HardwareMap hardwareMap, String motor, double maxSpeed, double accel, double motorStep) {

        // Get hardwareMap to access components.
        super(hardwareMap);

        // Get motors and servos.
        this.motor = super.get(DcMotor.class, motor);

        // Configure motors and servos.
//        this.motor.setDirection(DcMotorSimple.Direction.FORWARD);
//        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Save metrics.
        this.maxSpeed = maxSpeed;
        this.accel = accel;
        this.motorStep = motorStep;
    }

    public TurnAngle(HardwareMap hardwareMap, DcMotor motor, double maxSpeed, double accel, double motorStep) {

        // Get hardwareMap to access components.
        super(hardwareMap);

        // Get motors and servos.
        this.motor = motor;

        // Configure motors and servos.
//        this.motor.setDirection(DcMotorSimple.Direction.FORWARD);
//        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Save metrics.
        this.maxSpeed = maxSpeed;
        this.accel = accel;
        this.motorStep = motorStep;
    }

    public void turnTo(double speed, int angle, Telemetry telemetry) {

//        angle = angle * 1680/360;



//        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setTargetPosition((int)(angle*this.motorStep/360));
        this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("pos", Double.toString(this.motor.getCurrentPosition()));
        telemetry.addData("pos", (int)(angle*this.motorStep/360));
        telemetry.update();
//        this.motor.setPower(.01);
        this.motor.setPower(speed);

        while (this.motor.isBusy()){
            telemetry.addData("pos", Double.toString(this.motor.getCurrentPosition()));
            telemetry.addData("pos", (int)(angle*this.motorStep/360));
            telemetry.addData("pos", "still here");
            telemetry.addData("pos", this.motor.getPower());
            telemetry.update();
        }

        this.motor.setPower(0);

//        this.motor.setPower(-1);

//        if (this.motor.getCurrentPosition() <= LIFT_STEPS && this.motor.getCurrentPosition() <= angle &&
//                this.motor.getCurrentPosition() >= 0) {
//            accelMotor(this.motor, speed);
//        } else {
//            this.stop();
//        }

//        this.motor.setPower(speed);
//        this.motor.setTargetPosition(angle * 1680/360);
//        this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        stop();
    }


    public void raise(double speed) {

        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (this.motor.getCurrentPosition() <= LIFT_STEPS &&
                this.motor.getCurrentPosition() >= 0) {
            accelMotor(this.motor, speed);
        } else {
            this.stop();
        }
    }

    public void stop() {

        this.motor.setPower(0);
    }

    public void up() {

        while (this.motor.getCurrentPosition() >= 0 &&
                this.motor.getCurrentPosition() <= LIFT_STEPS) {

            this.motor.setPower(this.maxSpeed);
        }

        this.stop();
    }

    public void down() {

        while (this.motor.getCurrentPosition() >= 0 &&
                this.motor.getCurrentPosition() <= LIFT_STEPS) {

            this.motor.setPower(-this.maxSpeed);
        }

        this.stop();
    }
    public double getMotorPosition(){
        return this.motor.getCurrentPosition();
    }



    /**
     * A trapezoidal acceleration control for the motors, to avoid abrupt accelerations/decelerations.
     *
     * @param motor:    The DcMotor to which the power is applied.
     * @param setSpeed: The final velocity [-1, 1].
     */
    private void accelMotor(DcMotor motor, double setSpeed) {

        double newSpeed = 0;

        // If current power is bigger than what the acceleration limit allows (either way).
        if (this.accel < Math.abs(motor.getPower() - setSpeed)) {
            // If current power is lower than what it should be.
            if (motor.getPower() - setSpeed < 0)
                newSpeed = motor.getPower() + this.accel;
                // If current power is higher than what it should be.
            else
                newSpeed = motor.getPower() - this.accel;
        }
        // If current power is within acceleration limits.
        else {
            newSpeed = setSpeed;
        }

        newSpeed = clip(newSpeed, -this.maxSpeed, this.maxSpeed);
        motor.setPower(newSpeed);
    }
}
