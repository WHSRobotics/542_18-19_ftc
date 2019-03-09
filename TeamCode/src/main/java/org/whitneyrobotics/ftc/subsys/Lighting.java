package org.whitneyrobotics.ftc.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by ivanm on 1/19/2018.
 */

public class Lighting {
    private DcMotorSimple led;

    public Lighting(HardwareMap lightMap) {
        led = lightMap.get(DcMotorSimple.class, "led");
    }

    public void operateLED(double power) {
        led.setPower(power);
    }
}