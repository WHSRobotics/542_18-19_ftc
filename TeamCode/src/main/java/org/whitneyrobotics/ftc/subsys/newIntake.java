package org.whitneyrobotics.ftc.subsys;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.whitneyrobotics.ftc.lib.util.Toggler;

public class newIntake {

    CRServo intakeServo;
    Servo clearenceServo;

    Toggler clearanceToggler;

    final double INTAKE_SPEED = 0.7;

    public newIntake(HardwareMap newIntakeMap){
        clearenceServo = newIntakeMap.servo.get("clearenceServo");
        intakeServo = newIntakeMap.crservo.get("intakeServo");
        clearanceToggler = new Toggler(2);
    }

    public void operateIntake (boolean gamepadInput1, boolean gamepadInput2){
        if (gamepadInput1){
            intakeServo.setPower(INTAKE_SPEED);
        }else if (gamepadInput2){
            intakeServo.setPower(-INTAKE_SPEED);
        }else{
            intakeServo.setPower(0);
        }

    }

    public void operateIntakeClearence(boolean gamepadInput){
        clearanceToggler.changeState(gamepadInput);
        if (clearanceToggler.currentState() == 0){
            clearenceServo.setPosition(.542);
        }else if (clearanceToggler.currentState() == 1){
            clearenceServo.setPosition(.865);
        }
    }

    public void setIntakeServoPower(double power){
        intakeServo.setPower(power);
    }


}
