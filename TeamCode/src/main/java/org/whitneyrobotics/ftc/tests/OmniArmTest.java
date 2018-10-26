package org.whitneyrobotics.ftc.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.whitneyrobotics.ftc.subsys.OmniArm;
import org.whitneyrobotics.ftc.subsys.WHSRobotImpl;
@TeleOp(name = "OmniArmTester", group = "test")
public class OmniArmTest extends OpMode{

    DcMotor switchMotor;
    OmniArm omniArm;
    @Override
    public void init() {
        switchMotor = hardwareMap.dcMotor.get("switchMotor");
        omniArm = new OmniArm(hardwareMap);
    }

    @Override
    public void loop(){
      if (gamepad1.right_bumper){
      switchMotor.setPower(.75);
      }else if (gamepad1.left_bumper){
          switchMotor.setPower(-.75);
      }else {
          switchMotor.setPower(0);
      }


    }
}
