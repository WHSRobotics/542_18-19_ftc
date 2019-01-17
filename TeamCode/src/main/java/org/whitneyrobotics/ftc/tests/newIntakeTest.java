package org.whitneyrobotics.ftc.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.subsys.newIntake;

@TeleOp (name = "newIntake's Intake Test")
public class newIntakeTest extends OpMode {
    newIntake jeff;
    @Override
    public void init() {
    jeff = new newIntake(hardwareMap);
    }

    @Override
    public void loop() {
        jeff.operateIntake(gamepad1.right_bumper, gamepad1.left_bumper);
        jeff.operateIntakeClearence(gamepad1.y);

    }
}
