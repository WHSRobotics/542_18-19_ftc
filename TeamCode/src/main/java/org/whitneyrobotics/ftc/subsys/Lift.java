package org.whitneyrobotics.ftc.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.whitneyrobotics.ftc.lib.subsys.MotorSubsystem;

public class Lift implements MotorSubsystem {

    public DcMotor liftMotor;
    private DigitalChannel limitSwitch;

    public enum LiftPosition {
        STORED, IN_LATCH, ABOVE_LATCH, FINAL
    }

    // STORED, IN_LATCH, ABOVE_LATCH, FINAL
    private final int[] LIFT_POSITIONS = {0, 4600, 5400, 542};
    private final int STORED_HEIGHT = LIFT_POSITIONS[LiftPosition.STORED.ordinal()];
    private final int IN_LATCH_HEIGHT = LIFT_POSITIONS[LiftPosition.IN_LATCH.ordinal()];
    private final int ABOVE_HEIGHT = LIFT_POSITIONS[LiftPosition.ABOVE_LATCH.ordinal()];
    private final int FINAL_HEIGHT = LIFT_POSITIONS[LiftPosition.FINAL.ordinal()];
    private final int LIFT_HEIGHT_THRESHOLD = 50;
    private final double LIFT_POWER = 0.8;

    int liftUpRobotState = 0;
    int bringDownRobotState = 0;
    int bringDownHookState = 0;
    private LiftPosition currentLiftPosition;

    public Lift(HardwareMap liftMap) {
        liftMotor = liftMap.dcMotor.get("liftMotor");
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        limitSwitch = liftMap.digitalChannel.get("limitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    public void setLiftMotorPower(double power) {
        liftMotor.setPower(power);
        if (power != 0.0) {
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void bringDownRobot(boolean gamepadInput) {
        switch (bringDownRobotState) {
            case 0:
                if (gamepadInput) {
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    bringDownRobotState = 1;
                    bringDownHookState = 0;
                    liftUpRobotState = 0;
                }
                break;
            case 1:
                setLiftPosition(LiftPosition.ABOVE_LATCH);
                bringDownRobotState = 2;
                break;
            case 2:
                if (liftMotor.getCurrentPosition() > (ABOVE_HEIGHT - LIFT_HEIGHT_THRESHOLD)) {
                    bringDownRobotState = 3;
                }
                break;
            case 3:
                liftMotor.setPower(0.0);
                currentLiftPosition = LiftPosition.ABOVE_LATCH;
                bringDownRobotState = 0;
                break;
        }
    }

    public void bringDownHook(boolean gamepadInput) {
        switch (bringDownHookState) {
            case 0:
                if (gamepadInput) {
                    liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    bringDownHookState = 1;
                    bringDownRobotState = 0;
                    liftUpRobotState = 0;
                }
                break;
            case 1:
                liftMotor.setPower(-LIFT_POWER);
                bringDownHookState = 2;
                break;
            case 2:
                if (getDigitalTouch()) {
                    bringDownHookState = 3;
                }
                break;
            case 3:
                liftMotor.setPower(0.0);
                currentLiftPosition = LiftPosition.STORED;
                liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                bringDownHookState = 0;
        }
    }

    public void liftUpRobot(boolean gamepadInput) {
        switch (liftUpRobotState) {
            case 0:
                if (gamepadInput) {
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftUpRobotState = 1;
                    bringDownRobotState = 0;
                    bringDownHookState = 0;
                }
                break;
            case 1:
                setLiftPosition(LiftPosition.IN_LATCH);
                liftUpRobotState = 2;
                break;
            case 2:
                if (liftMotor.getCurrentPosition() > (IN_LATCH_HEIGHT - LIFT_HEIGHT_THRESHOLD)) {
                    liftUpRobotState = 3;
                }
                break;
            case 3:
                setLiftPosition(LiftPosition.FINAL);
                break;
        }
    }

    public void setLiftPosition(LiftPosition liftPosition) {
        liftMotor.setTargetPosition(LIFT_POSITIONS[liftPosition.ordinal()]);
        liftMotor.setPower(LIFT_POWER);
    }

    public LiftPosition getCurrentLiftPosition() {
        return currentLiftPosition;
    }

    public int getEncoderPos() {
        return liftMotor.getCurrentPosition();
    }

    public int getTargetPos() {
        return liftMotor.getTargetPosition();
    }

    public boolean getDigitalTouch() {
        return !limitSwitch.getState();
    }

    @Override
    public void setRunMode(DcMotor.RunMode runMode) {
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        liftMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

}
