package org.whitneyrobotics.ftc.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.lib.util.Toggler;
import org.whitneyrobotics.ftc.subsys.MarkerDrop;
import org.whitneyrobotics.ftc.subsys.ParticleFlipper;


/**
 * Created by ivanm on 11/4/2017.
 */
@TeleOp(name = "Particle Flipper Test", group = "tests")
public class ParticleFlipperTest extends OpMode {

    ParticleFlipper particleFlipper;
    Toggler toggler;
    long i;

    @Override
    public void init() {
        particleFlipper = new ParticleFlipper(hardwareMap);
        toggler = new Toggler(200);
    }

    @Override
    public void loop() {
        toggler.changeState(gamepad1.dpad_up, gamepad1.dpad_down);

        i++;
        if (i%10 == 0) {
            if (gamepad1.a) toggler.setState(toggler.currentState() + 1);
        }
        particleFlipper.operateParticleFlipper(toggler.currentState()/200f);
        telemetry.addData("Position: ", toggler.currentState()/200f);
    }
}
