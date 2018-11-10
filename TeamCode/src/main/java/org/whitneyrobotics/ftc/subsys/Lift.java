package org.whitneyrobotics.ftc.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.whitneyrobotics.ftc.lib.subsys.MotorSubsystem;

public class Lift implements MotorSubsystem {

    private DcMotor liftMotor;
    private DigitalChannel limitSwitch;

    public enum LiftPosition {
        STORED, IN_LATCH, ABOVE_LATCH, FINAL
    }

    // STORED, IN_LATCH, ABOVE_LATCH, FINAL
    private final int[] LIFT_POSITIONS = {-68, 4680, 4830, 542};
    private final int STORED_HEIGHT = LIFT_POSITIONS[LiftPosition.STORED.ordinal()];
    private final int IN_LATCH_HEIGHT = LIFT_POSITIONS[LiftPosition.IN_LATCH.ordinal()];
    private final int ABOVE_HEIGHT = LIFT_POSITIONS[LiftPosition.ABOVE_LATCH.ordinal()];
    private final int FINAL_HEIGHTT = LIFT_POSITIONS[LiftPosition.FINAL.ordinal()];
    private final int LIFT_HEIGHT_THRESHOLD =50;
    private final double LIFT_POWER = 0.8;
    boolean hasLiftReachedTargetHeight = false;
    boolean liftInProgress = false;
    public boolean isRobotDown = false;
    public boolean isHookDown = false;
    int liftState = 0;
    int dropState = 0;
    int hookState = 0;

    public Lift(HardwareMap liftMap) {
        liftMotor = liftMap.dcMotor.get("liftMotor");
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limitSwitch = liftMap.digitalChannel.get("limitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    public void setLiftMotorPower(double power){
        liftMotor.setPower(power);
        if(power > 0.01) {
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void liftUpRobot(boolean gamepadInput) {

        switch (liftState){
            case 0:
                if(gamepadInput){
                    liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    liftState = 1;
                }
                break;
            case 1:
                liftMotor.setPower(LIFT_POWER);
                liftMotor.setTargetPosition(IN_LATCH_HEIGHT);
                liftState = 2;
                break;
            case 2:
                if(liftMotor.getCurrentPosition() > (IN_LATCH_HEIGHT - LIFT_HEIGHT_THRESHOLD)){
                    liftState = 3;
                }
                break;
            case 3:
                liftMotor.setTargetPosition(STORED_HEIGHT);
                liftState = 4;
                break;
            case 4:
                if(liftMotor.getCurrentPosition() < (STORED_HEIGHT + LIFT_HEIGHT_THRESHOLD)){
                    liftState = 5;
                }
                break;
            case 5:
                liftMotor.setPower(0.0);
                liftState = 0;
                break;
        }
    }


    public void bringDownRobot(boolean gamepadInput){

        switch (dropState) {
            case 0:
                isRobotDown = false;
                if (gamepadInput) {
                    dropState = 1;
                }
                break;
            case 1:
                liftMotor.setPower(LIFT_POWER);
                liftMotor.setTargetPosition(ABOVE_HEIGHT);
                dropState = 2;
                break;
            case 2:
                if (liftMotor.getCurrentPosition() > (ABOVE_HEIGHT - LIFT_HEIGHT_THRESHOLD)) {
                    dropState = 3;
                }
                break;
            case 3:
                isRobotDown = true;
                liftMotor.setPower(0.0);
                dropState = 0;
                break;
        }
    }

    public void bringDownHook(boolean gamepadInput){

        switch (hookState) {
            case 0:
                isHookDown = false;
                if (gamepadInput) {
                    hookState = 1;
                }
                break;
            case 1:
                liftMotor.setPower(LIFT_POWER);
                liftMotor.setTargetPosition(STORED_HEIGHT);
                hookState = 2;
                break;
            case 2:
                if (liftMotor.getCurrentPosition() < (STORED_HEIGHT + LIFT_HEIGHT_THRESHOLD)) {
                    hookState = 3;
                }
                break;
            case 3:
                isHookDown = true;
                liftMotor.setPower(0.0);
                hookState = 0;
                break;
        }
    }

    public void setLiftPosition(LiftPosition liftPosition) {
        liftMotor.setTargetPosition(LIFT_POSITIONS[liftPosition.ordinal()]);
        liftMotor.setPower(LIFT_POWER);
    }

    @Override
    public void setRunMode(DcMotor.RunMode runMode) {
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        // required for lift to implement MotorSubsystem
    }

    @Override
    public double getAbsPowerAverage() {
        return 0;
    }

    public void resetEncoderValue() {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean getDigitalTouch() { return limitSwitch.getState(); }
}
