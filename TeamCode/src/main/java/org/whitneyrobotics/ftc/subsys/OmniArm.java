package org.whitneyrobotics.ftc.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OmniArm {

    public DcMotor linearSlideMotor;

    public OmniArm(HardwareMap armMap) {
        linearSlideMotor = armMap.dcMotor.get("motorLS");
    }
}
