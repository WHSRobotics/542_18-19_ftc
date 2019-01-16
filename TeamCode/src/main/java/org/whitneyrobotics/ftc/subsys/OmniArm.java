package org.whitneyrobotics.ftc.subsys;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.whitneyrobotics.ftc.lib.util.Toggler;

public class OmniArm {

    public DcMotor intakeMotor;
    public DcMotor extendMotor;
    public DcMotor pivotMotor;

    private CRServo leftSweep;
    private CRServo rightSweep;

    private DigitalChannel omniLimitSwitch;

    public enum ExtendPosition {
        RETRACTED, EXTENDED
    }

    public enum PivotPosition {
        STORED, ROOM_FOR_LIFT, OUTTAKE, INTAKE
    }

    private final double INTAKE_POWER = 1.0;
    private final double OUTTAKE_POWER = -1.0;

    //RETRACTED, EXTENDED
    private  final int[] EXTEND_POSITIONS = {0, 2100};
    private final int RETRACTED_LENGTH = EXTEND_POSITIONS[ExtendPosition.RETRACTED.ordinal()];
    private final int EXTENDED_LENGTH = EXTEND_POSITIONS[ExtendPosition.EXTENDED.ordinal()];

    //STORED, ROOM_FOR_LIFT, OUTTAKE, INTAKE
    private final int[] PIVOT_POSITIONS = {0, 320, 290, 2020};
    private final int STORED_MODE = PIVOT_POSITIONS[PivotPosition.STORED.ordinal()];
    private final int ROOM_FOR_LIFT_MODE = PIVOT_POSITIONS[PivotPosition.ROOM_FOR_LIFT.ordinal()];
    private final int OUTTAKE_MODE = PIVOT_POSITIONS[PivotPosition.OUTTAKE.ordinal()];
    private final int INTAKE_MODE = PIVOT_POSITIONS[PivotPosition.INTAKE.ordinal()];

    private final double PIVOT_POWER = 0.35;
    private PivotPosition currentPivotPosition;

    Toggler extensionToggler = new Toggler(2);
    Toggler pivotToggler = new Toggler(2);

    public int limitSwitchResetState = 0;
    public int operateModeSwitch = 0;
    public boolean isLimitSwitchResetInProgress = false;

    public OmniArm(HardwareMap armMap) {
        intakeMotor = armMap.dcMotor.get("intakeMotor");
        extendMotor = armMap.dcMotor.get("extendMotor");
        pivotMotor = armMap.dcMotor.get("pivotMotor");
        leftSweep = armMap.crservo.get("lSweepServo");
        rightSweep = armMap.crservo.get("rSweepServo");
        omniLimitSwitch = armMap.digitalChannel.get("omniLimitSwitch");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void operateIntake(boolean intakeGamepadInput, boolean outtakeGamepadInput) {
        if (intakeGamepadInput) {
            intakeMotor.setPower(INTAKE_POWER);
        } else if (outtakeGamepadInput) {
            intakeMotor.setPower(OUTTAKE_POWER);
        } else {
            intakeMotor.setPower(0.0);
        }
    }

    public void operateSweeps(boolean intakeGamepadInput, boolean outtakeGamepadInput) {
        if (intakeGamepadInput) {
            leftSweep.setPower(-1);
            rightSweep.setPower(1);
        } else if (outtakeGamepadInput) {
            leftSweep.setPower(1);
            rightSweep.setPower(-1);
        } else {
            leftSweep.setPower(0);
            rightSweep.setPower(0);
        }
    }

    public void operateExtend(boolean gamepadInput) {
        extensionToggler.changeState(gamepadInput);
        if (gamepadInput) {
            extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (extensionToggler.currentState() == 0) {
            extendMotor.setTargetPosition(RETRACTED_LENGTH);
            extendMotor.setPower(0.50);
        } else if (extensionToggler.currentState() == 1) {
            extendMotor.setTargetPosition(EXTENDED_LENGTH);
            extendMotor.setPower(0.50);
        }
    }

    public void operatePivot(boolean gamepadInput) {
        pivotToggler.changeState(gamepadInput);
        if (gamepadInput) {
            pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (pivotToggler.currentState() == 0) {
            setPivotPosition(PivotPosition.INTAKE);
            currentPivotPosition = PivotPosition.INTAKE;
        } else if (pivotToggler.currentState() == 1) {
            setPivotPosition(PivotPosition.OUTTAKE);
            currentPivotPosition = PivotPosition.OUTTAKE;
        }
    }

    public void makeRoomForLift (boolean gamepadInput) {
        if (gamepadInput) {
            pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        setPivotPosition(PivotPosition.ROOM_FOR_LIFT);
    }

    public void storePivot (boolean gamepadInput) {
        if (gamepadInput) {
            pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        setPivotPosition(PivotPosition.STORED);
    }

    public void setPivotPosition(PivotPosition pivotPosition) {
        pivotMotor.setTargetPosition(PIVOT_POSITIONS[pivotPosition.ordinal()]);
        pivotMotor.setPower(PIVOT_POWER);
    }

    public void setExtendMotorPower(double power) {
        extendMotor.setPower(power);
        if (power != 0) {
            extendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void setPivotMotorPower(double power) {
        pivotMotor.setPower(power);
        if (power != 0) {
            pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void limitSwitchReset(boolean gamepadInput) {
        switch (limitSwitchResetState) {
            case 0:
                if (gamepadInput) {
                    limitSwitchResetState = 1;
                    isLimitSwitchResetInProgress = true;
                    pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                } else {
                    pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                break;
            case 1:
                pivotMotor.setPower(-PIVOT_POWER);
                limitSwitchResetState = 2;
                break;
            case 2:
                if (getDigitalTouch()) {
                    limitSwitchResetState = 3;
                }
                break;
            case 3:
                pivotMotor.setPower(0.0);
                isLimitSwitchResetInProgress = false;
                currentPivotPosition = PivotPosition.STORED;
                pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                limitSwitchResetState = 0;
                //INTAKE_MODE = 2150;
        }
    }

    public void resetEncoders() {
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean getDigitalTouch() {
        return !omniLimitSwitch.getState();
    }

    public int getPivotTogglerState() {
        return pivotToggler.currentState();
    }

}
