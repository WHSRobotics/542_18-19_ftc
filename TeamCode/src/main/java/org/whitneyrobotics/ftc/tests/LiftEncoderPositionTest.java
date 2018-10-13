package org.whitneyrobotics.ftc.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.whitneyrobotics.ftc.lib.util.Toggler;
import org.whitneyrobotics.ftc.subsys.Lift;

@TeleOp(name = "LiftTest", group = "tests")
public class LiftEncoderPositionTest extends OpMode {

    DcMotor lift;
    Toggler tog = new Toggler(20);
    int i = 0;
    int encoderPos = 0;


    @Override
    public void init() {
        lift = hardwareMap.dcMotor.get("liftMotor");
        tog.setState(10);
    }

    @Override
    public void loop() {
        i++;
        if(gamepad1.a ){
            encoderPos++;
        }
        else if(gamepad1.b){
            encoderPos--;
        }
        if(gamepad1.x){
            lift.setPower(.666);
        }else if (gamepad1.y){
            lift.setPower(-.666);
        }else{
            lift.setPower(0);
        }
        tog.changeState(gamepad1.dpad_up, gamepad1.dpad_down);
        lift.setTargetPosition(encoderPos + tog.currentState() - 10);
        telemetry.addData("motorthing", encoderPos);
        telemetry.addData("motoradfasdf", lift.getCurrentPosition());
    }
}
