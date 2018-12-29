package org.whitneyrobotics.ftc.subsys;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.whitneyrobotics.ftc.lib.util.Toggler;

public class OmniArm {

    public DcMotor extendMotor;
    public DcMotor intakeMotor;
    public DcMotor switchMotor;

    private CRServo leftSweep;
    private CRServo rightSweep;

    private DigitalChannel omniLimitSwitch;

    private final int EXTEND_LENGTH = 2300;
    private final int RETRACT_LENGTH = 0;
    private final int RESET_MODE = 0;
    private final int INTAKE_MODE = 2150;
    private final int STORED_MODE = 300;
    private final int OUTTAKE_MODE = 900;
    private final double INTAKE_SPEED = 1.0;
    private final double OUTTAKE_SPEED = -1.0;

    Toggler extensionToggler = new Toggler(2);
    Toggler switchToggler = new Toggler(2);
    Toggler storeToggler = new Toggler(2);

    public int omniArmLimitSwitchResetState = 0;
    public int operateModeSwitch = 0;
    public boolean isLimitSwitchResetInProgress = false;



    public OmniArm(HardwareMap armMap) {
        extendMotor = armMap.dcMotor.get("extendMotor");
        intakeMotor = armMap.dcMotor.get("intakeMotor");
        switchMotor = armMap.dcMotor.get("switchMotor");
        leftSweep = armMap.crservo.get("lSweepServo");
        rightSweep = armMap.crservo.get("rSweepServo");
        omniLimitSwitch = armMap.digitalChannel.get("omniLimitSwitch");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        switchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        switchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        switchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void operateIntake(boolean gamepadInputIntake, boolean gamepadInputOuttake) {
        if (gamepadInputIntake) {
            intakeMotor.setPower(INTAKE_SPEED);
        } else if (gamepadInputOuttake) {
            intakeMotor.setPower(OUTTAKE_SPEED);
        } else {
            intakeMotor.setPower(0.0);
        }
    }

    public void operateSweepServos(boolean gamepadInputIntake, boolean gamepadInputOuttake) {
        if (gamepadInputIntake) {
            leftSweep.setPower(-1);
            rightSweep.setPower(1);
        } else if (gamepadInputOuttake) {
            leftSweep.setPower(1);
            rightSweep.setPower(-1);
        } else {
            leftSweep.setPower(0);
            rightSweep.setPower(0);
        }
    }


    public void operateExtension(boolean gamepadInput) {
        extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extensionToggler.changeState(gamepadInput);
        if (extensionToggler.currentState() == 0) {
            extendMotor.setTargetPosition(RETRACT_LENGTH);
            extendMotor.setPower(0.50);
        } else if (extensionToggler.currentState() == 1) {
            extendMotor.setTargetPosition(EXTEND_LENGTH);
            extendMotor.setPower(0.50);
        }
    }

    public void operateModeSwitch(boolean gamepadInput) {
        switch (operateModeSwitch) {
            case 0:
                if (gamepadInput) {
                    switchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    switchToggler.changeState(gamepadInput);
                }

                if (switchToggler.currentState() == 0) {
                    switchMotor.setTargetPosition(STORED_MODE);
                    switchMotor.setPower(.35);
                    operateModeSwitch = 1;
                    switchToggler.setState(1);
                } else if (switchToggler.currentState() == 1) {
                    switchMotor.setTargetPosition(INTAKE_MODE);
                    switchMotor.setPower(.35);
                    switchToggler.setState(0);
                    operateModeSwitch = 1;
                }
                break;

            case 1:
                if (gamepadInput){
                    operateModeSwitch = 0;
                }
                break;

        }
    }

    public void storeOmniArm(boolean gamepadInput) {
        switchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        storeToggler.changeState(gamepadInput);
        if (storeToggler.currentState() == 1) {
            switchMotor.setTargetPosition(STORED_MODE);
            switchMotor.setPower(0.25);
        }
    }

    public void resetOmniArm(boolean gamepadInput) {
        switchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        switchMotor.setTargetPosition(RESET_MODE);
        switchMotor.setPower(0.25);
    }

    public void setSwitchMotorPower(double power) {
        switchMotor.setPower(power);
        if (power > 0.01) {
            switchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void setExtendMotorPower(double power) {
        extendMotor.setPower(power);
        if (power > 0.01) {
            extendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void limitSwitchReset(boolean gamepadInput){
        switch (omniArmLimitSwitchResetState){
            case 0:
                if (gamepadInput){
                    omniArmLimitSwitchResetState = 1;
                    isLimitSwitchResetInProgress = true;
                }
                 break;
            case 1:
                switchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                omniArmLimitSwitchResetState = 2;
                isLimitSwitchResetInProgress = true;
                break;
            case 2:

                if (getOmniDigitalTouch()){
                    switchMotor.setPower(0);
                    isLimitSwitchResetInProgress = false;
                    omniArmLimitSwitchResetState = 0;
                }else{
                    switchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    switchMotor.setPower(-.35);
                }
        }

    }
    public void resetEncoders() {
        switchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean getOmniDigitalTouch(){
        return !omniLimitSwitch.getState();
    }
}