package org.whitneyrobotics.ftc.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.whitneyrobotics.ftc.lib.util.Toggler;
@TeleOp(name = "MotorPositionTest", group = "tests")
public class MotorArmPositionTest extends OpMode {
    DcMotor motor;
    Toggler tog = new Toggler(3);
    @Override
    public void init() {
        motor = hardwareMap.dcMotor.get("motor");
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(0);

    }

    @Override
    public void loop() {
        if(tog.currentState() == 0){
            motor.setTargetPosition(0);
        }
        if(tog.currentState() == 1){
            motor.setTargetPosition(224);
        }
        if(tog.currentState() == 2){
            motor.setTargetPosition(672);
        }
        tog.changeState(gamepad1.a);
        if(gamepad1.b){
            motor.setPower(1.0);
        }
        else {
            motor.setPower(0.0);
        }
    }
}
