package org.whitneyrobotics.ftc.autoop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.whitneyrobotics.ftc.subsys.Lift;

@Autonomous(name = "Pub Display Servo", group = "auto")
public class PubDisplayServo extends OpMode {
    CRServo pubDisplayServo;

    @Override
    public void init() {
        pubDisplayServo = hardwareMap.crservo.get("Pub Display Servo");
    }

    @Override
    public void loop() {
        pubDisplayServo.setPower(0.05);
    }
}
