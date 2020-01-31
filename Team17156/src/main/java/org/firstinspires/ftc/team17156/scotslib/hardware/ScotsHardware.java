/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.team17156.scotslib.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p>
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class ScotsHardware {
    /* Public OpMode members. */
    public DcMotor leftFrontDrive;
    public DcMotor rightFrontDrive;
    public DcMotor rightBackDrive;
    public DcMotor leftBackDrive;
    public DcMotor leftIntakeMotor;
    public DcMotor rightIntakeMotor;
    public DcMotor liftingMotor;
    public DcMotor sideLiftingMotor;
    public Servo wristServo;
    public Servo grabbingServo;
    public Servo sideGrabbingServo;
    public Servo sideWristServo;
    public Servo intakeServo;
    public Servo leftFoundationServo ;
    public Servo rightFoundationServo;
    public Servo capstoneServo;

    public static final double MID_SERVO = 0.5;
    public static final double ARM_UP_POWER = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;

    /* local OpMode members. */
    HardwareMap hardwareMap;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public void HardwarePushbot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hardwareMap) {
        // Save reference to Hardware map
        this.hardwareMap = hardwareMap;

        // Define and Initialize Motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "front_left_motor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right_motor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "back_left_motor");
        leftBackDrive = hardwareMap.get(DcMotor.class, "back_right_motor");
        leftIntakeMotor = hardwareMap.get(DcMotor.class, "left_intake_motor");
        rightIntakeMotor = hardwareMap.get(DcMotor.class, "right_intake_motor");
        liftingMotor = hardwareMap.get(DcMotor.class, "lifting_motor");
        sideLiftingMotor = hardwareMap.get(DcMotor.class, "side_lifting_motor");

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        wristServo = hardwareMap.get(Servo.class, "wrist_servo");
        grabbingServo = hardwareMap.get(Servo.class, "grabbing_servo");
        sideGrabbingServo = hardwareMap.get(Servo.class, "side_grabbing_servo");
        sideWristServo = hardwareMap.get(Servo.class, "side_wrist_servo");
        intakeServo = hardwareMap.get(Servo.class, "intake_servo");
        leftFoundationServo = hardwareMap.get(Servo.class, "left_foundation_servo");
        rightFoundationServo = hardwareMap.get(Servo.class, "right_foundation_servo");
        capstoneServo = hardwareMap.get(Servo.class, "capstone_servo");
    }
}

