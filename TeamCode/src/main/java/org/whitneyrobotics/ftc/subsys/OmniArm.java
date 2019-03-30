package org.whitneyrobotics.ftc.subsys;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.whitneyrobotics.ftc.lib.util.Toggler;

import java.util.Map;

public class OmniArm {

    //Motors
    public DcMotor extendMotor;
    public DcMotor pivotMotor;
    private DcMotor intakeMotor;
    //Servos
    //LimitSwitch
   // private DigitalChannel omniLimitSwitch;

    //Encoder Position Enums
    public enum ExtendPosition {
        RETRACTED,OUTTAKE, INTAKE
    }
    public enum PivotPosition {
        STORED, ROOM_FOR_LIFT, OUTTAKE, INTAKE, INTERMEDIATE
    }



    //Powers and Thresholds
    private final double INTAKE_POWER = 0.95;
    private final double EXTEND_POWER = 0.50;
    private final double PIVOT_POWER = 0.35;
    private final double PIVOT_THRESHOLD = 50;

    //RETRACTED, EXTENDED
    private  final int[] EXTEND_POSITIONS = {300,3000, 4000};
    private final int OUTTAKE_LENGTH = EXTEND_POSITIONS[ExtendPosition.OUTTAKE.ordinal()];
    private final int RETRACTED_LENGTH = EXTEND_POSITIONS[ExtendPosition.RETRACTED.ordinal()];
    private final int INTAKE_LENGTH = EXTEND_POSITIONS[ExtendPosition.INTAKE.ordinal()];

    //STORED, ROOM_FOR_LIFT, OUTTAKE, INTAKE, Intermediate
    private final int[] PIVOT_POSITIONS = {0, 320, 1700, 0, 500};
    private final int STORED_MODE = PIVOT_POSITIONS[PivotPosition.STORED.ordinal()];
    private final int ROOM_FOR_LIFT_MODE = PIVOT_POSITIONS[PivotPosition.ROOM_FOR_LIFT.ordinal()];
    private final int OUTTAKE_MODE = PIVOT_POSITIONS[PivotPosition.OUTTAKE.ordinal()];
    private final int INTAKE_MODE = PIVOT_POSITIONS[PivotPosition.INTAKE.ordinal()];
    private final int INTERMEDIATE_MODE = PIVOT_POSITIONS[PivotPosition.INTERMEDIATE.ordinal()];

    //biases
    private int armPivotBias = 0;
    private int armPivotBiasAmount = 50;
    private int armExtendBiasAmount = 50;
    private int armExtendBias = 0;

    private PivotPosition currentPivotPosition = PivotPosition.STORED;

    Toggler extensionToggler = new Toggler(2);
    Toggler pivotToggler = new Toggler(2);

    public int limitSwitchResetState = 0;
    public int operateModeSwitch = 0;



    public OmniArm(HardwareMap armMap) {

        extendMotor = armMap.dcMotor.get("extendMotor");
        pivotMotor = armMap.dcMotor.get("pivotMotor");
        intakeMotor = armMap.dcMotor.get("intakeMotor");
        //omniLimitSwitch = armMap.digitalChannel.get("omniLimitSwitch");
        //z   omniLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        extendMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }



    public void operateIntake (boolean gamepadInput1, boolean gamepadInput2, boolean gamepadInputSlow){
        if (gamepadInput1){
            intakeMotor.setPower(INTAKE_POWER);
        }else if (gamepadInput2){
            intakeMotor.setPower(-INTAKE_POWER);
        }else if (gamepadInputSlow){
            intakeMotor.setPower(-INTAKE_POWER/1.75);
        }
        else{
            intakeMotor.setPower(0);
        }

    }


    public void operateExtend(boolean gamepadInput) {
        extensionToggler.changeState(gamepadInput);
        if (gamepadInput) {
            extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extendMotor.setPower(EXTEND_POWER);
        }
        if (extensionToggler.currentState() == 0) {
            extendMotor.setTargetPosition(RETRACTED_LENGTH + armExtendBias);
        } else if (extensionToggler.currentState() == 1) {
            extendMotor.setTargetPosition(INTAKE_LENGTH + armExtendBias);
        }
    }



    public void operatePivot(boolean gamepadInput, boolean gamepadInputExtendClearance) {

        pivotToggler.changeState(gamepadInput);
        if (gamepadInput || gamepadInputExtendClearance) {
            pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivotMotor.setPower(PIVOT_POWER);
        }
        if (gamepadInputExtendClearance){
            pivotMotor.setTargetPosition(INTERMEDIATE_MODE + armPivotBias);
            currentPivotPosition = PivotPosition.INTERMEDIATE;
        }
        else if (pivotToggler.currentState() == 0 && gamepadInput) {
            pivotMotor.setTargetPosition(OUTTAKE_MODE + armPivotBias);
            currentPivotPosition = PivotPosition.OUTTAKE;
        } else if (pivotToggler.currentState() == 1 && gamepadInput) {
            pivotMotor.setTargetPosition(INTAKE_MODE + armPivotBias);
            currentPivotPosition = PivotPosition.INTAKE;
        }
    }


    // for use in Auto
    public void setPivotPosition(PivotPosition pivotPosition) {
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setTargetPosition(PIVOT_POSITIONS[pivotPosition.ordinal()]);
        pivotMotor.setPower(PIVOT_POWER);
        if (Math.abs(pivotMotor.getCurrentPosition() - PIVOT_POSITIONS[pivotPosition.ordinal()]) < PIVOT_THRESHOLD) {
            currentPivotPosition = pivotPosition;
        }
    }

    public void limitSwitchReset(boolean gamepadInput) {
        switch (limitSwitchResetState) {
            case 0:
                if (gamepadInput) {
                    limitSwitchResetState = 1;
                    pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
                currentPivotPosition = PivotPosition.STORED;
                pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                limitSwitchResetState = 0;
                //INTAKE_MODE = 2150;
        }
    }

    public PivotPosition getCurrentPivotPosition() {
        return currentPivotPosition;
    }

    public void operateExtendManual(boolean gamepadInput1, double gamepadInput2) {

        if (gamepadInput1) {
            extendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extendMotor.setPower(gamepadInput2);
        } else {
            extendMotor.setPower(0.0);
        }
    }

    public void operatePivotManual(boolean gamepadInput1, double gamepadInput2) {
        if(gamepadInput1) {
            pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pivotMotor.setPower(gamepadInput2);
        } else {
            pivotMotor.setPower(0.0);
        }
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

    public void resetEncoders() {
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean getDigitalTouch() {
        return false;
    }

    public void operateArmPivotBias(boolean gamepadInputUp, boolean gamepadInputDown){
        if (gamepadInputUp){
            armPivotBias = armPivotBiasAmount;
        }else if (gamepadInputDown){
            armPivotBias = -armPivotBiasAmount;
        }
        else{
            armPivotBias = 0;
        }
    }

    public void operateArmExtendBias(boolean gamepadInputUp, boolean gamepadinputdown){
        if (gamepadInputUp){
            armExtendBias= armExtendBiasAmount;
        }else if (gamepadinputdown){
            armExtendBias = -armExtendBiasAmount;
        }else{
            armExtendBias =0;
        }
    }



    /*public void makeRoomForLift (boolean gamepadInput) {
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

    public int getPivotTogglerState() {
        return pivotToggler.currentState();
    }*/

}
