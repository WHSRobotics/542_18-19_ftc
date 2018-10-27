package org.whitneyrobotics.ftc.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.whitneyrobotics.ftc.lib.util.Toggler;

public class OmniArm {

    public DcMotor extendMotor;
    public DcMotor intakeMotor;
    public DcMotor switchMotor;

    private final int EXTEND_LENGTH = 500;
    private final int CONTRACT_LENGTH = 0;
    private final int INTAKE_MODE = 200;
    private final int OUTTAKE_MODE = -100;
    private final double INTAKE_SPEED = 0.75;
    private final double OUTTAKE_SPEED = -1.0;

    Toggler toggler = new Toggler(2);

    public OmniArm(HardwareMap armMap) {
        extendMotor = armMap.dcMotor.get("extendMotor");
        intakeMotor = armMap.dcMotor.get("intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        switchMotor = armMap.dcMotor.get("switchMotor");
        extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        switchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void operateIntake(boolean gamepadInputIntake, boolean gamepadInputOuttake){
        if(gamepadInputIntake){
            intakeMotor.setPower(INTAKE_SPEED);
        }
        else if(gamepadInputOuttake){
            intakeMotor.setPower(OUTTAKE_SPEED);
        }
        else {
            intakeMotor.setPower(0.0);
        }
    }

    public void extendIntake(){
        extendMotor.setTargetPosition(EXTEND_LENGTH);
        extendMotor.setPower(.5);
    }

    public void contractIntake(){
        extendMotor.setTargetPosition(CONTRACT_LENGTH);
        extendMotor.setPower(-.5);
    }

    public void operateModeSwitch(boolean gamepadInput1, boolean gamepadInput2){
        //TODO set orientation of switch motors
        if (gamepadInput1){
                switchMotor.setTargetPosition(OUTTAKE_MODE);
                switchMotor.setPower(.75);
        }
        if (gamepadInput2){
                switchMotor.setTargetPosition(INTAKE_MODE);
                switchMotor.setPower(-.75);

        }
    }
}