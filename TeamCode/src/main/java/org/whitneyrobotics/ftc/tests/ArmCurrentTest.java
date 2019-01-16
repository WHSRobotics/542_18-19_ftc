package org.whitneyrobotics.ftc.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.whitneyrobotics.ftc.lib.util.Toggler;

@TeleOp(name = "ArmCurrentTest", group = "tests")
public class ArmCurrentTest extends OpMode {

    DcMotor switchMotor;
    Toggler brakeTog = new Toggler(2);

    @Override
    public void init() {
        switchMotor = hardwareMap.dcMotor.get("pivotMotor");

    }

    @Override
    public void loop() {
        if(gamepad1.y){
            switchMotor.setPower(0.0);
            switchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            switchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if(gamepad1.x){
            switchMotor.setTargetPosition(switchMotor.getCurrentPosition());
        }
        if (gamepad1.b){
            switchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            switchMotor.setPower(1.0);
        }
        brakeTog.changeState(gamepad1.a);
        if(brakeTog.currentState() == 0){
            switchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        else {
            switchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        telemetry.addData("CurrentEncoderPos", switchMotor.getCurrentPosition());
        telemetry.addData("CommandedEncoderPos", switchMotor.getTargetPosition());
        telemetry.addData("Brake On?", brakeTog.currentState()==0 ? "Yes" : "No");

    }
}
