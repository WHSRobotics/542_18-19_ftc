package org.whitneyrobotics.ftc.subsys;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.whitneyrobotics.ftc.lib.util.Toggler;

import java.util.Map;

public class Jeffrey {

    CRServo intakeServo1;
    CRServo intakeServo2;
    Servo clearenceServo;

    Toggler jeffMouthToggler;
    Toggler clearanceToggler;

    final int INTAKE_SPEED = 1;

    public Jeffrey(HardwareMap jeffMap){
        intakeServo1 = jeffMap.crservo.get("intakeServo2");
        clearenceServo = jeffMap.servo.get("clearenceServo");
        intakeServo2 = jeffMap.crservo.get("intakeServo1");
        jeffMouthToggler = new Toggler(4);
        clearanceToggler = new Toggler(2);
    }

    public void operateJeff (boolean gamepadInput){
        jeffMouthToggler.changeState(gamepadInput);

        if (jeffMouthToggler.currentState() == 0){
            intakeServo1.setPower(INTAKE_SPEED);
            intakeServo2.setPower(INTAKE_SPEED);
        }else if (jeffMouthToggler.currentState() == 1){
            intakeServo1.setPower(0);
            intakeServo2.setPower(0);
        }else if (jeffMouthToggler.currentState() == 2){
            intakeServo1.setPower(-INTAKE_SPEED);
            intakeServo2.setPower(-INTAKE_SPEED);
        }else if (jeffMouthToggler.currentState() == 3){
            intakeServo1.setPower(0);
            intakeServo2.setPower(0);
        }
    }

    public void operateClearence(boolean gamepadInput){
        clearanceToggler.changeState(gamepadInput);
        if (clearanceToggler.currentState() == 0){
            clearenceServo.setPosition(.25);
        }else if (clearanceToggler.currentState() == 1){
            clearenceServo.setPosition(.8);
        }
    }

    public void setIntakeServo1Power(double power){
        intakeServo1.setPower(power);
    }

    public void setIntakeServo2Power(double power){
        intakeServo2.setPower(power);
    }
}
