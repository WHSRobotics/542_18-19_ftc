package org.whitneyrobotics.ftc.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.whitneyrobotics.ftc.lib.subsys.MotorSubsystem;

public class Lift implements MotorSubsystem {

    private DcMotor liftMotor;

    public enum LiftPosition {
        STORED, IN_LATCH, ABOVE_LATCH, FINAL
    }

    //STORED, IN_LATCH, ABOVE_LATCH, FINAL
    private final int[] LIFT_POSITIONS = {-68, 4680, 4720, 542};
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
    }

    public void setLiftMotorPower(double power){
        liftMotor.setPower(power);
    }

    public void liftUpRobot(boolean gamepadInput) {

        switch (liftState){
            case 0:
                if(gamepadInput){
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

        /*
        if(gamepadInput || liftInProgress) {
            liftInProgress = true;

            if (!hasLiftReachedTargetHeight) {
                liftMotor.setTargetPosition(IN_LATCH_HEIGHT);
                liftMotor.setPower(1.0);
            }
            if (liftMotor.getCurrentPosition() > (IN_LATCH_HEIGHT - LIFT_HEIGHT_THRESHOLD) || hasLiftReachedTargetHeight) {
                liftMotor.setTargetPosition(STORED_HEIGHT);
                hasLiftReachedTargetHeight = true;
                liftMotor.setPower(1.0);
            } else {
                liftMotor.setPower(0.0);
                liftInProgress = false;
                hasLiftReachedTargetHeight = false;
            }

        }*/
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

        /*if(gamepadInput || isRobotDown){
        if(!hasLiftReachedTargetHeight){
            liftMotor.setTargetPosition(ABOVE_HEIGHT);
            liftMotor.setPower(1.0);
        }if (liftMotor.getCurrentPosition() > (ABOVE_HEIGHT - LIFT_HEIGHT_THRESHOLD) || hasLiftReachedTargetHeight){
            liftMotor.setTargetPosition(FINAL_HEIGHTT);
            liftMotor.setPower(1.0);
        }else{
            liftMotor.setPower(0);
        }
    }*/
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

        /*if(gamepadInput || isHookDown){
        if(!hasLiftReachedTargetHeight){
            liftMotor.setTargetPosition(STORED_HEIGHT);
            liftMotor.setPower(1.0);
        }
        if(liftMotor.getCurrentPosition() < (ABOVE_HEIGHT-LIFT_HEIGHT_THRESHOLD) || hasLiftReachedTargetHeight){
            liftMotor.setPower(0);
        }
    }*/
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

    }

    @Override
    public double getAbsPowerAverage() {
        return 0;
    }
}
