package org.whitneyrobotics.ftc.subsys;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.whitneyrobotics.ftc.lib.util.Toggler;

public class newIntake {

    CRServo intakeServo1;
    CRServo intakeServo2;
    Servo clearenceServo;

    Toggler intakeSpeedToggler;
    Toggler clearanceToggler;

    final double INTAKE_SPEED = 0.7;

    public newIntake(HardwareMap newIntakeMap){
        intakeServo1 = newIntakeMap.crservo.get("intakeServo2");
        clearenceServo = newIntakeMap.servo.get("clearenceServo");
        intakeServo2 = newIntakeMap.crservo.get("intakeServo1");
        intakeSpeedToggler = new Toggler(2);
        clearanceToggler = new Toggler(2);
    }

    public void operateIntake (boolean gamepadInput, boolean gamepadInput2){
        intakeSpeedToggler.changeState(gamepadInput);
        if (gamepadInput2){
            intakeServo2.setPower(0);
            intakeServo1.setPower(0);
        }else if (intakeSpeedToggler.currentState() == 0){
            intakeServo1.setPower(INTAKE_SPEED);
            intakeServo2.setPower(-INTAKE_SPEED);
        }else if (intakeSpeedToggler.currentState() == 1){
            intakeServo1.setPower(-INTAKE_SPEED);
            intakeServo2.setPower(INTAKE_SPEED);
        }

    }

    public void operateIntakeClearence(boolean gamepadInput){
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
