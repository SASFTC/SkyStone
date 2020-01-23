//package org.firstinspires.ftc.team17156.scotslib.hardware.drivetrain;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//
//import static android.os.SystemClock.sleep;
//import static com.qualcomm.robotcore.util.Range.clip;
//
//public class TurnAngle {
//
//    /* Fields */
//    private HardwareMap hardwareMap;
//    private DcMotor motor;
//
//    private double accel;
//    private double maxSpeed;
//    private ElapsedTime runtime = new ElapsedTime();
//
//    // Keep track of some metrics.
//    private double speed = 0;               //  -1 <=     speed     <= 1
//    private double angle = 0;               // -pi <=     angle     <= pi
//    private double rotation = 0;            //  -1 <=    rotation   <= 1
//    BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
//
//
//
//    /* Methods */
//
//    /**
//     * Constructor for the Mecanum Drive class. Given an instance of all four motors,
//     * it handles all the necessary math and logic to drive the mecanum Drivetrain.
//     *
//     * @param hardwareMap: The reference to the HardwareMap in the OpClass.
//     * @param motor:       The motor.
//     * @param accel:       The acceleration limit of the robot.
//     * @param maxSpeed:    The maximum speed of the robot.
//     */
//    public TurnAngle(HardwareMap hardwareMap, String motor, double accel, double maxSpeed,
//                     boolean invertedDrive) {
//        // Get HardwareMap
//        this.hardwareMap = hardwareMap;
//
//        // Set the motors' instances.
//        this.motor = this.hardwareMap.get(DcMotor.class, motor);
//
//        // Set the motor orientation.
//        if (!invertedDrive) {
//            this.motor.setDirection(DcMotorSimple.Direction.FORWARD);
//        } else {
//            this.motor.setDirection(DcMotorSimple.Direction.REVERSE);
//        }
//
//        // Set ZeroPowerBehavior to BRAKE, so that any forces acting on the robot will not cause movement.
//        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        // Set other params.
//        this.accel = accel;
//        this.maxSpeed = maxSpeed;
//
//    }
//
//    /**
//     * Set accel, maxSpeed to default ( 0.5 & 1 ), not inverted.
//     * @param hardwareMap
//     * @param motor
//     * @param angle
//     */
//
//
//    /**
//     * The main driving method.
//     * Sets the appropriate motor powers given the speed, angle, and rotation speed.
//     *
//     * @param speed:    The magnitude of the velocity [-1, 1].
//     * @param angle:    The direction of the velocity [-pi, pi]. The angle is zero when going forward,
//     *                  negative clockwise, and positive counterclockwise.
//     * @param rotation: The rotational speed [-1, 1]. Clockwise is positive, and counterclockwise is negative.
//     */
//    public void drive(double speed, double angle, double rotation) {
//
//        // Constrain speed to max speed.
//        this.speed = clip(speed, -this.maxSpeed, this.maxSpeed);
//        this.angle = angle;
//        this.rotation = clip(-rotation, -this.maxSpeed, this.maxSpeed);
//
//        // Calculate the motor's speed.
//        double v1 = this.speed * Math.sin(this.angle + Math.PI / 4) - this.rotation;
//
//        // Apply the desired power to each motor.
//        this.motor.setPower(clip(v1, -1, 1));
//    }
//
//
//    public void run(double turnAngle) {
//        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        double speed = 0.5;
//        double oldDegreesLeft = turnAngle;
//        double scaledSpeed = speed;
//        double targetHeading = angles.firstAngle + turnAngle;
//        double oldAngle = angles.firstAngle;
//        if (targetHeading < -180) {
//            targetHeading += 360;
//        }
//        if (targetHeading > 180) {
//            targetHeading -= 360;
//        }
//        double degreesLeft = ((int) (Math.signum(angles.firstAngle - targetHeading) + 1) / 2) * (360 - Math.abs(angles.firstAngle - targetHeading)) + (int) (Math.signum(targetHeading - angles.firstAngle) + 1) / 2 * Math.abs(angles.firstAngle - targetHeading);
//        runtime.reset();
//        while (opModeIsActive() && runtime.seconds() < timeoutS && degreesLeft > 1 && oldDegreesLeft - degreesLeft >= 0) { //check to see if we overshot target
//            scaledSpeed = degreesLeft / (100 + degreesLeft) * speed;
//            if (scaledSpeed > 1) {
//                scaledSpeed = .1;
//            }
//            robot.leftBack.setPower(scaledSpeed * 1.3); //extra power to back wheels
//            robot.rightBack.setPower(-1 * scaledSpeed * 1.3); //due to extra weight
//            robot.leftFront.setPower(scaledSpeed);
//            robot.rightFront.setPower(-1 * scaledSpeed);
//            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            oldDegreesLeft = degreesLeft;
//            degreesLeft = ((int) (Math.signum(angles.firstAngle - targetHeading) + 1) / 2) * (360 - Math.abs(angles.firstAngle - targetHeading)) + (int) (Math.signum(targetHeading - angles.firstAngle) + 1) / 2 * Math.abs(angles.firstAngle - targetHeading);
//            if (Math.abs(angles.firstAngle - oldAngle) < 1) {
//                speed *= 1.1;
//            } //bump up speed to wheels in case robot stalls before reaching target
//            oldAngle = angles.firstAngle;
//        }
//        stopWheels(); //our helper method to set all wheel motors to zero
//        sleep(250); //small pause at end of turn
//    }
//
//
//    /**
//     * Sets the motor power to zero.
//     */
//    public void stop() {
//
//        // Set all motors to zero power.
//        this.motor.setPower(0);
//    }
//
//}
