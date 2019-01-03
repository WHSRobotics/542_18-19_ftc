package org.whitneyrobotics.ftc.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.whitneyrobotics.ftc.subsys.OmniArm;
import org.whitneyrobotics.ftc.subsys.TileRunner;
import org.whitneyrobotics.ftc.subsys.WHSRobotImpl;
@TeleOp(name = "OmniArmTest", group = "tests")
public class OmniArmTest extends OpMode{

    WHSRobotImpl robot;
    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);    }

    @Override
    public void loop(){

      robot.omniArm.operateIntake(gamepad1.right_trigger>0.1, gamepad1.left_trigger>0.1);
        if (gamepad1.x){
            robot.omniArm.operateModeSwitch(gamepad1.x);
        }else {
            robot.omniArm.limitSwitchReset(gamepad1.b);
        }
      robot.omniArm.operateSweepServos(gamepad1.left_bumper, gamepad1.right_bumper);
      robot.omniArm.operateExtension(gamepad1.a);
      telemetry.addData("OmniLimit State" , robot.omniArm.getOmniDigitalTouch());
      telemetry.addData("OmniSwitchSwitchState", robot.omniArm.operateModeSwitch);


    }
}
