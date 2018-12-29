package org.whitneyrobotics.ftc.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.whitneyrobotics.ftc.subsys.OmniArm;
import org.whitneyrobotics.ftc.subsys.WHSRobotImpl;
@TeleOp(name = "OmniArmTest", group = "tests")
public class OmniArmTest extends OpMode{

    OmniArm omniArm;
    @Override
    public void init() {
        omniArm = new OmniArm(hardwareMap);
    }

    @Override
    public void loop(){

      omniArm.operateIntake(gamepad1.right_trigger>0.1, gamepad1.left_trigger>0.1);

          omniArm.limitSwitchReset(gamepad1.a);
          omniArm.operateModeSwitch(gamepad1.y);

      omniArm.operateSweepServos(gamepad1.left_bumper, gamepad1.right_bumper);
      omniArm.operateExtension(gamepad1.x);
      telemetry.addData("OmniLimit State" , omniArm.getOmniDigitalTouch());
      telemetry.addData("OmniSwitchSwitchState", omniArm.operateModeSwitch);


    }
}
