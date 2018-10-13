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
    int encoderPos = 0;


    @Override
    public void init() {
        lift = hardwareMap.dcMotor.get("liftMotor");
    }

    @Override
    public void loop() {
        if(gamepad1.a ){
            encoderPos++;
        }
        else if(gamepad1.b){
            encoderPos--;
        }
        if(gamepad1.x){
            lift.setPower(.666);
        }else if (gamepad1.y) {
            lift.setPower(-.666);
        }
        else if(gamepad1.right_bumper) {
            lift.setPower(1.0);
        }else if(gamepad1.right_trigger > 0.10){
            lift.setPower(-1.0);
        }else{
            lift.setPower(0);
        }
        lift.setTargetPosition(encoderPos);
        telemetry.addData("Motor Current Pos", encoderPos);
        telemetry.addData("Motor Target Pos", lift.getCurrentPosition());
    }
}
