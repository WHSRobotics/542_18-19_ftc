package org.whitneyrobotics.ftc.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by ivanm on 10/27/2018.
 */
@TeleOp(name = "omniArmEncodersTest", group = "tests")
public class OmniArmEncodersTest extends OpMode {

    private DcMotor switchMotor;
    private DcMotor extendMotor;

    @Override
    public void init() {
        switchMotor = hardwareMap.dcMotor.get("switchMotor");
        extendMotor = hardwareMap.dcMotor.get("extendMotor");

        switchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        switchMotor.setPower(0.0);
        extendMotor.setPower(0.0);
        telemetry.addData("Switch Pos", switchMotor.getCurrentPosition());
        telemetry.addData("Extend     Pos", extendMotor.getCurrentPosition());
    }
}
                        