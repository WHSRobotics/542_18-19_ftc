package org.whitneyrobotics.ftc.subsys;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Potentiometer {
    public AnalogInput potentiometer;

    public Potentiometer(HardwareMap hardwareMap){
        potentiometer = hardwareMap.analogInput.get("potentiometer");
    }

    public double getPotentiometer (){
        return potentiometer.getVoltage();
    }
}
