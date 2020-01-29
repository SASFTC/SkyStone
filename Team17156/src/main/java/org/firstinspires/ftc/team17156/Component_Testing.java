package org.firstinspires.ftc.team17156;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.team17156.scotslib.hardware.component.motor.DcMotorControllerSA;
import org.firstinspires.ftc.team17156.scotslib.hardware.component.motor.DcMotorSA;

@TeleOp(name = "Component Testing", group = "Testing")
public class Component_Testing extends OpMode {

    DcMotorSA motor;
    DcMotorControllerSA motorController;

    @Override
    public void init() {

        this.motor = new DcMotorSA(hardwareMap.get(DcMotorImplEx.class, "lifting_motor"), DcMotorSA.Mode.BY_ANGLE);

        this.motorController = new DcMotorControllerSA();
        this.motorController.addMotor(this.motor);
        this.motorController.start();


//        this.motor.runToPosition(550 * 1680 / 360);
//        this.motor.setTargetPower(0.5);
    }

    @Override
    public void loop() {

//        this.motor.setTargetPower(1);

        this.motor.runToPosition(550 * 1680 / 360);
        this.motor.setTargetPower(0.8);


        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        this.motor.runToPosition(0);
        this.motor.setTargetPower(0.8);

        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }

    @Override
    public void stop() {
        this.motorController.stop();
    }
}
