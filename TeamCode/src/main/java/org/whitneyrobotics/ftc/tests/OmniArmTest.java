package org.whitneyrobotics.ftc.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.lib.util.Toggler;
import org.whitneyrobotics.ftc.subsys.WHSRobotImpl;

@TeleOp(name = "OmniArmTest", group = "tests")
public class OmniArmTest extends OpMode {

    WHSRobotImpl robot;
    Toggler armTog = new Toggler(2);
    int i = 0;

    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
    }

    @Override
    public void loop() {

        robot.omniArm.operateIntake(gamepad2.right_bumper, gamepad2.left_bumper);
        robot.omniArm.operateIntakeClearence(gamepad2.left_trigger>0.01);

        armTog.changeState(gamepad2.dpad_up);
        if (armTog.currentState() == 0) {
            if (gamepad2.back) {
                robot.omniArm.resetEncoders();
            }
            robot.omniArm.operateExtendManual(gamepad2.left_stick_button, gamepad2.left_stick_y);
            robot.omniArm.operatePivotManual(gamepad2.right_stick_button, gamepad2.right_stick_y);
} else if (armTog.currentState() == 1) {
            robot.omniArm.operatePivot(gamepad2.x, gamepad2.y);
            robot.omniArm.operateExtend(gamepad2.a);
        }
        telemetry.addData("OmniLimit State", robot.omniArm.getDigitalTouch());
        telemetry.addData("OmniSwitchSwitchState", robot.omniArm.operateModeSwitch);

        i++;
        telemetry.addData("i", i);


    }
}
