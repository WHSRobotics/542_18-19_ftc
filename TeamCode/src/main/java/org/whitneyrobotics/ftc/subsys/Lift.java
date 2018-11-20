package org.whitneyrobotics.ftc.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.whitneyrobotics.ftc.lib.subsys.MotorSubsystem;

public class Lift implements MotorSubsystem {

    public DcMotor liftMotor;
    private DigitalChannel limitSwitch;
    public DistanceSensor distancer;

    public enum LiftPosition {
        STORED, IN_LATCH, ABOVE_LATCH, FINAL
    }

    public enum LiftState {
        MANUAL_OVERRIDE, START, STORED_TO_ABOVE_LATCH, WAITING_FOR_DRIVETRAIN, ABOVE_LATCH_TO_STORED, STANDING_BY_FOR_END_GAME, STORED_TO_IN_LATCH, IN_LATCH_TO_FINAL, END
    }

    // STORED, IN_LATCH, ABOVE_LATCH, FINAL
    private final int[] LIFT_POSITIONS = {-68, 4600, 5240, 542};
    private final int STORED_HEIGHT = LIFT_POSITIONS[LiftPosition.STORED.ordinal()];
    private final int IN_LATCH_HEIGHT = LIFT_POSITIONS[LiftPosition.IN_LATCH.ordinal()];
    private final int ABOVE_HEIGHT = LIFT_POSITIONS[LiftPosition.ABOVE_LATCH.ordinal()];
    private final int FINAL_HEIGHT = LIFT_POSITIONS[LiftPosition.FINAL.ordinal()];
    private final int LIFT_HEIGHT_THRESHOLD = 50;
    private final double LIFT_POWER = 0.8;
    boolean hasLiftReachedTargetHeight = false;
    boolean liftInProgress = false;
    int sensorLiftInLatch = 500;

    int liftUpRobotState = 0;
    int bringDownRobotState = 0;
    int bringDownHookState = 0;
    private LiftState liftState;
    boolean liftApproached = false;
    int sensorLiftState;

    public Lift(HardwareMap liftMap) {
        liftMotor = liftMap.dcMotor.get("liftMotor");
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        limitSwitch = liftMap.digitalChannel.get("limitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        //liftState = LiftState.START;
        distancer = liftMap.get(DistanceSensor.class, "distanceSensor");
    }

    public void setLiftMotorPower(double power){
        liftMotor.setPower(power);
        if(power > 0.01) {
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        liftState = LiftState.MANUAL_OVERRIDE;
    }

    public void bringDownRobot(boolean gamepadInput){

        switch (bringDownRobotState) {
            case 0:
                //liftState = LiftState.START;
                if (gamepadInput) {
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    bringDownRobotState = 1;
                }
                break;
            case 1:
                liftMotor.setPower(LIFT_POWER);
                liftMotor.setTargetPosition(ABOVE_HEIGHT);
                liftState = LiftState.STORED_TO_ABOVE_LATCH;
                bringDownRobotState = 2;
                break;
            case 2:
                if (liftMotor.getCurrentPosition() > (ABOVE_HEIGHT - LIFT_HEIGHT_THRESHOLD)) {
                    bringDownRobotState = 3;
                }
                break;
            case 3:
                liftMotor.setPower(0.0);
                bringDownRobotState = 0;
                liftState = LiftState.WAITING_FOR_DRIVETRAIN;
                break;
        }
    }

    public void bringDownHook(boolean gamepadInput){

        switch (bringDownHookState) {
            case 0:
                if (gamepadInput) {
                    liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    bringDownHookState = 1;
                }
                break;
            case 1:
                liftMotor.setPower(-LIFT_POWER);
                //liftMotor.setTargetPosition(STORED_HEIGHT);
                liftState = LiftState.ABOVE_LATCH_TO_STORED;
                bringDownHookState = 2;
                break;
            case 2:
                if (!limitSwitch.getState()) {
                    liftMotor.setPower(0.0);
                    liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    liftState = LiftState.STANDING_BY_FOR_END_GAME;
                    bringDownHookState = 0;
                }
                break;

        }
    }

    public void liftUpRobot(boolean gamepadInput) {

        switch (liftUpRobotState){
            case 0:
                if(gamepadInput){
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftUpRobotState = 1;
                }
                break;
            case 1:
                liftMotor.setPower(LIFT_POWER);
                liftMotor.setTargetPosition(IN_LATCH_HEIGHT);
                liftState = LiftState.STORED_TO_IN_LATCH;
                liftUpRobotState = 2;
                break;
            case 2:
                if(liftMotor.getCurrentPosition() > (IN_LATCH_HEIGHT - LIFT_HEIGHT_THRESHOLD)){
                    liftUpRobotState = 3;
                }
                break;
            case 3:
                liftMotor.setTargetPosition(FINAL_HEIGHT);
                liftState = LiftState.IN_LATCH_TO_FINAL;
                liftUpRobotState = 4;
                break;
            case 4:
                if(liftMotor.getCurrentPosition() < (FINAL_HEIGHT + LIFT_HEIGHT_THRESHOLD)){
                    liftUpRobotState = 5;
                }
                break;
            case 5:
                liftMotor.setPower(0.0);
                liftState = LiftState.END;
                liftUpRobotState = 0;
                break;
        }
    }

    public LiftState getLiftState(){
        return liftState;
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
        liftMotor.setZeroPowerBehavior(zeroPowerBehavior);
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

    public void sensorLift(boolean gamepadInput) {
        switch (sensorLiftState) {
            case 0:
                if (gamepadInput) {
                    if (!liftApproached) {
                            liftMotor.setPower(.66);
                        }

                    if (distancer.getDistance(DistanceUnit.MM) < 10) {
                        liftApproached = true;
                        liftMotor.setPower(0);
                        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        sensorLiftState++;
                    }

                }
                break;

            case 2:
                liftMotor.setTargetPosition(sensorLiftInLatch);
                sensorLiftState++;


            case 3:
                liftMotor.setPower(.66);
                if(liftMotor.getCurrentPosition()> sensorLiftInLatch - LIFT_HEIGHT_THRESHOLD ){
                    liftMotor.setPower(0);
                    sensorLiftState++;
                }
                break;

            case 4:
                liftMotor.setTargetPosition(FINAL_HEIGHT);
                liftMotor.setPower(0);
                if (getDigitalTouch()){
                    liftMotor.setPower(0);
                    sensorLiftState++;
                }
                break;
        }
    }
}
