package org.whitneyrobotics.ftc.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.whitneyrobotics.ftc.subsys.Jeffrey;

import java.util.Map;

@TeleOp (name = "Jeffrey's Intake Test")
public class newIntakeTest extends OpMode {

    Jeffrey jeff;
    @Override
    public void init() {
    jeff = new Jeffrey(hardwareMap);
    }

    @Override
    public void loop() {

        if (gamepad1.right_bumper){
            jeff.setIntakeServo1Power(1);
        }else if (gamepad1.left_bumper){
            jeff.setIntakeServo1Power(-1);
        }else{
            jeff.setIntakeServo1Power(0);
        }

        if (gamepad1.right_trigger>0.01){
            jeff.setIntakeServo2Power(1);
        }else if (gamepad1.left_trigger >0.01){
            jeff.setIntakeServo2Power(-1);
        }else{
            jeff.setIntakeServo2Power(0);
        }

    }
}
