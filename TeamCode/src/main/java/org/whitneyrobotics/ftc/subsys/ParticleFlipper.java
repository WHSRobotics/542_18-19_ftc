package org.whitneyrobotics.ftc.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.whitneyrobotics.ftc.lib.util.SimpleTimer;

public class ParticleFlipper {
    private Servo particleFlipperServo;
    private static final double[] particleFlipperPositions = {.205, .78}; // stored, extended
    public enum ParticleFlipperPosition {
        STORED, EXTENDED;
    }

    public ParticleFlipper(HardwareMap particleFlipperMap) {
        particleFlipperServo = particleFlipperMap.servo.get("particle flipper");
    }

    public void operateParticleFlipper(ParticleFlipperPosition particleFlipperPosition) {
        particleFlipperServo.setPosition(particleFlipperPositions[particleFlipperPosition.ordinal()]);
    }

    public void operateParticleFlipper(double position) {
        particleFlipperServo.setPosition(position);
    }
}
