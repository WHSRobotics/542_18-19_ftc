package org.whitneyrobotics.ftc.autoop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.whitneyrobotics.ftc.subsys.Lift;

@Autonomous(name = "Bring Down Hook", group = "auto")
public class BringDownHook extends OpMode {
    Lift lift;
    @Override
    public void init() {
        lift = new Lift(hardwareMap);
    }

    @Override
    public void loop() {
        lift.bringDownHook(true);
    }
}
