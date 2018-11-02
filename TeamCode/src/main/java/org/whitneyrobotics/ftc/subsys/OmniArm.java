package org.whitneyrobotics.ftc.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.whitneyrobotics.ftc.lib.util.Toggler;

public class OmniArm {

    public DcMotor extendMotor;
    public DcMotor intakeMotor;
    public DcMotor switchMotor;

    private final int EXTEND_LENGTH = 1154;
    private final int RETRACT_LENGTH = 0;
    private final int INTAKE_MODE = 2124;
    private final int OUTTAKE_MODE = 100;
    private final double INTAKE_SPEED = 0.75;
    private final double OUTTAKE_SPEED = -1.0;

    Toggler extensionToggler = new Toggler(2);
    Toggler switchToggler = new Toggler(2);

    public OmniArm(HardwareMap armMap) {
        extendMotor = armMap.dcMotor.get("extendMotor");
        intakeMotor = armMap.dcMotor.get("intakeMotor");
        switchMotor = armMap.dcMotor.get("switchMotor");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        switchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        switchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        switchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    public void operateExtension(boolean gamepadInput) {
        extensionToggler.changeState(gamepadInput);
        if(extensionToggler.currentState() == 0) {
            extendMotor.setTargetPosition(RETRACT_LENGTH);
            extendMotor.setPower(0.1);
        } else if (extensionToggler.currentState() == 1) {
            extendMotor.setTargetPosition(EXTEND_LENGTH);
            extendMotor.setPower(0.1);
        }
    }

    public void operateModeSwitch(boolean gamepadInput){
        switchToggler.changeState(gamepadInput);
        if(switchToggler.currentState() == 0) {
            switchMotor.setTargetPosition(OUTTAKE_MODE);
            switchMotor.setPower(0.1);
        } else if (switchToggler.currentState() == 1) {
            switchMotor.setTargetPosition(INTAKE_MODE);
            switchMotor.setPower(0.1);
        }
    }
}