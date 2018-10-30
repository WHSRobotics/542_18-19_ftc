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
    private final int[] LIFT_POSITIONS = {0, 3900, 4200, 0};
    private final int STORED_HEIGHT = LIFT_POSITIONS[LiftPosition.STORED.ordinal()];
    private final int IN_LATCH_HEIGHT = LIFT_POSITIONS[LiftPosition.IN_LATCH.ordinal()];
    private final int ABOVE_HEIGHT = LIFT_POSITIONS[LiftPosition.ABOVE_LATCH.ordinal()];
    private final int LIFT_HEIGHT_THRESHOLD = 200;
    private final double LIFT_POWER = 0.8;
    boolean hasLiftReachedTargetHeight = false;

    public Lift(HardwareMap liftMap) {
        liftMotor = liftMap.dcMotor.get("liftMotor");
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setLiftMotorPower(double power){
        liftMotor.setPower(power);
    }

    public void liftUpRobot() {
        if (!hasLiftReachedTargetHeight) {
            liftMotor.setTargetPosition(IN_LATCH_HEIGHT);
            liftMotor.setPower(1.0);
        }
        if (liftMotor.getCurrentPosition() > (IN_LATCH_HEIGHT - LIFT_HEIGHT_THRESHOLD) || hasLiftReachedTargetHeight) {
            liftMotor.setTargetPosition(STORED_HEIGHT);
            hasLiftReachedTargetHeight = true;
            liftMotor.setPower(1.0);
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

    }

    @Override
    public double getAbsPowerAverage() {
        return 0;
    }
}
