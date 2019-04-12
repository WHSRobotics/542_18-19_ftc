package org.whitneyrobotics.ftc.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.whitneyrobotics.ftc.lib.util.SimpleTimer;
import org.whitneyrobotics.ftc.lib.util.Toggler;

public class OmniArm {

    //Motors
    public DcMotor extendMotor;
    public DcMotor pivotMotor;
    private DcMotor intakeMotor;
    //Servos
    public Servo clearanceServo;
    //LimitSwitch
   // private DigitalChannel omniLimitSwitch;

    //Encoder Position Enums
    public enum ExtendPosition {
        RETRACTED, INTERMEDIATE, OUTTAKE, INTAKE
    }
    public enum PivotPosition {
        STORED, AUTO_INTERMEDIATE, AUTO_INTAKE, OUTTAKE, INTAKE, INTERMEDIATE
    }

    public enum ClearancePosition{
        INTAKE, OUTTAKE
    }



    //Powers and Thresholds
    private final double INTAKE_POWER = 0.95;
    private final double EXTEND_POWER = 1.0;
    private final double PIVOT_POWER = .4;
    private final double PIVOT_SLOW_POWER = .12;
    private final double EXTEND_THRESHOLD = 50;
    private final double PIVOT_THRESHOLD = 50;

    //RETRACTED, INTERMEDIATE, OUTTAKE, INTAKE
    private  final int[] EXTEND_POSITIONS = {600, 5700, 5230, 7600};
    private final int RETRACTED_LENGTH = EXTEND_POSITIONS[ExtendPosition.RETRACTED.ordinal()];
    private final int INTERMEDIATE_LENGTH = EXTEND_POSITIONS[ExtendPosition.INTERMEDIATE.ordinal()];
    private final int OUTTAKE_LENGTH = EXTEND_POSITIONS[ExtendPosition.OUTTAKE.ordinal()];
    private final int INTAKE_LENGTH = EXTEND_POSITIONS[ExtendPosition.INTAKE.ordinal()];

    //STORED, AUTO_INTERMEDIATE, AUTO_INTAKE, OUTTAKE, INTAKE, INTERMEDIATE
    private final int[] PIVOT_POSITIONS = {0, 150, -295, 1602, -425, 600};
    private final int STORED_MODE = PIVOT_POSITIONS[PivotPosition.STORED.ordinal()];
    private final int AUTO_INTERMEDIATE = PIVOT_POSITIONS[PivotPosition.AUTO_INTERMEDIATE.ordinal()];
    private final int AUTO_INTAKE = PIVOT_POSITIONS[PivotPosition.AUTO_INTAKE.ordinal()];
    private final int OUTTAKE_MODE = PIVOT_POSITIONS[PivotPosition.OUTTAKE.ordinal()];
    private final int INTAKE_MODE = PIVOT_POSITIONS[PivotPosition.INTAKE.ordinal()];
    private final int INTERMEDIATE_MODE = PIVOT_POSITIONS[PivotPosition.INTERMEDIATE.ordinal()];

    //Intake, Outtake
    private final double [] CLEARANCE_POSITIONS = {.965,.265};
    private final double INTAKE_CLEARANCE = CLEARANCE_POSITIONS[ClearancePosition.INTAKE.ordinal()];
    private final double OUTTAKE_CLEARANCE = CLEARANCE_POSITIONS[ClearancePosition.OUTTAKE.ordinal()];

    //biases
    private int armPivotBias = 0;
    private int armPivotBiasAmount = 300;
    private int armExtendBiasAmount = 254;
    private int armExtendBias = 0;

    private PivotPosition currentPivotPosition = PivotPosition.STORED;
    private ExtendPosition currentExtendPosition = ExtendPosition.RETRACTED;

    Toggler extensionToggler = new Toggler(2);
    Toggler pivotToggler = new Toggler(2);

    public int limitSwitchResetState = 0;
    public int operateModeSwitch = 0;
    public int operatePivotState = 0;

    SimpleTimer pivotWaitTimer;
    public static final double PIVOT_WAIT_DELAY = 1.245;

    public OmniArm(HardwareMap armMap) {

        extendMotor = armMap.dcMotor.get("extendMotor");
        pivotMotor = armMap.dcMotor.get("pivotMotor");
        intakeMotor = armMap.dcMotor.get("intakeMotor");
        clearanceServo = armMap.servo.get("clearanceServo");
        //omniLimitSwitch = armMap.digitalChannel.get("omniLimitSwitch");
        //z   omniLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        extendMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        extendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pivotWaitTimer = new SimpleTimer();
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

    public void operateIntakeClearence(boolean gamepadInputIntake){
        if (gamepadInputIntake){
            clearanceServo.setPosition(OUTTAKE_CLEARANCE);
        }else {
            clearanceServo.setPosition(INTAKE_CLEARANCE);
        }
    }

    public void operateExtend(boolean gamepadInput) {
        extensionToggler.changeState(gamepadInput);
        if (gamepadInput) {
            extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extendMotor.setPower(EXTEND_POWER);
        }
        if (extensionToggler.currentState() == 0) {
            extendMotor.setTargetPosition(RETRACTED_LENGTH);
        } else if (extensionToggler.currentState() == 1) {
            if (currentPivotPosition == PivotPosition.INTAKE || currentPivotPosition == PivotPosition.INTERMEDIATE){
                extendMotor.setTargetPosition(OUTTAKE_LENGTH + armExtendBias);
            } else if (currentPivotPosition == PivotPosition.OUTTAKE && pivotMotor.getCurrentPosition() > 500){
                extendMotor.setTargetPosition(OUTTAKE_LENGTH + armExtendBias);
            }
        }
    }

    public void operateClearance(boolean gamepadInput){
        if (gamepadInput){
            clearanceServo.setPosition(OUTTAKE_CLEARANCE);
        }else {
            clearanceServo.setPosition(INTAKE_CLEARANCE);
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
            extendMotor.setTargetPosition(INTAKE_LENGTH);

        } else if (pivotToggler.currentState() == 1 && gamepadInput) {
            pivotMotor.setTargetPosition(INTAKE_MODE + armPivotBias);
            extendMotor.setTargetPosition(OUTTAKE_LENGTH);
            currentPivotPosition = PivotPosition.INTAKE;
            if(pivotMotor.getCurrentPosition() <200){
                pivotMotor.setPower(PIVOT_SLOW_POWER);
            }
            else if (pivotMotor.getCurrentPosition()>800 && pivotMotor.getCurrentPosition() <1000 && currentPivotPosition == PivotPosition.OUTTAKE)
                pivotMotor.setPower(PIVOT_SLOW_POWER);
        }   else if (currentPivotPosition == PivotPosition.OUTTAKE && pivotMotor.getCurrentPosition()>1000 ){
            pivotMotor.setPower(-PIVOT_SLOW_POWER);
        }
    }

    //WIP: new operatePivot with retract, then pivot, then extend

    public void newOperatePivot(boolean gamepadInput, boolean gamepadInputRetractExtend) {
        pivotToggler.changeState(gamepadInput);

        if (pivotToggler.currentState() == 1) {
            switch (operatePivotState) {
                case 0:
                    if (gamepadInput) {
                        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        operatePivotState = 1;
                    }
                    break;
                case 1:
                    setPivotPosition(PivotPosition.INTERMEDIATE);
                    if (currentPivotPosition == PivotPosition.INTERMEDIATE) {
                        operatePivotState = 2;
                    }
                    break;
                case 2:
                    setExtendPosition(ExtendPosition.OUTTAKE);
                    if (currentExtendPosition == ExtendPosition.OUTTAKE) {
                        pivotWaitTimer.set(PIVOT_WAIT_DELAY);
                        operatePivotState = 3;
                    }
                    break;
                case 3:
                    if(pivotWaitTimer.isExpired()) {
                        operatePivotState = 4;
                    }
                    break;
                case 4:
                    pivotMotor.setTargetPosition(OUTTAKE_MODE);
                    pivotMotor.setPower(PIVOT_POWER);
                    if (Math.abs(pivotMotor.getCurrentPosition() - OUTTAKE_MODE) < PIVOT_THRESHOLD) {
                        operatePivotState = 5;
                    }
                    break;
                case 5:
                    if(gamepadInputRetractExtend){
                        extendMotor.setTargetPosition(RETRACTED_LENGTH);
                        pivotMotor.setTargetPosition(INTERMEDIATE_MODE);
                        pivotMotor.setPower(PIVOT_POWER);
                    } else {
                        extendMotor.setTargetPosition(OUTTAKE_LENGTH + armExtendBias);
                    }
                    extendMotor.setPower(EXTEND_POWER);
            }
        }
        else if (pivotToggler.currentState() == 0 ) {
            operatePivotState = 0;
            if (gamepadInput) {
                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivotMotor.setPower(PIVOT_POWER);
                extendMotor.setPower(EXTEND_POWER);
            }
            extendMotor.setTargetPosition(OUTTAKE_LENGTH + armExtendBias);
            pivotMotor.setTargetPosition(INTAKE_MODE + armPivotBias);
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

    public void setExtendPosition(ExtendPosition extendPosition) {
        extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendMotor.setTargetPosition(EXTEND_POSITIONS[extendPosition.ordinal()]);
        extendMotor.setPower(EXTEND_POWER);
        if (Math.abs(extendMotor.getCurrentPosition() - EXTEND_POSITIONS[extendPosition.ordinal()]) < EXTEND_THRESHOLD) {
            currentExtendPosition = extendPosition;
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

    public ExtendPosition getCurrentExtendPosition() {
        return currentExtendPosition;
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

    public void operateArmExtendBias(double gamepadInput){
        armExtendBias = (int) (-(INTAKE_LENGTH - OUTTAKE_LENGTH) * gamepadInput);
    }

    public boolean clearedParticleFlipper() {
        if (pivotMotor.getCurrentPosition() > 320) {
            return true;
        }
        return false;
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
