package org.whitneyrobotics.ftc.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.whitneyrobotics.ftc.lib.subsys.MotorSubsystem;

public class Lift implements MotorSubsystem {

    private DcMotor liftMotor;
    private final int LIFT_HEIGHT = 3600;
    private final int FINAL_HEIGHT = 5376;
    private final int DOWN_HEIGHT = 0;
    private final int LIFT_HEIGHT_THRESHOLD = 400;
    boolean hasLiftReachedTargetHeight = false;

    enum LiftPositions{

    }

    public Lift(HardwareMap liftMap) {
        liftMotor = liftMap.dcMotor.get("liftMotor");
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setLiftMotorPower(double power){
        liftMotor.setPower(power);
    }

    public void liftUpRobot() {
        if (!hasLiftReachedTargetHeight) {
            liftMotor.setTargetPosition(LIFT_HEIGHT);
            liftMotor.setPower(1.0);
        }
        if (liftMotor.getCurrentPosition() > (LIFT_HEIGHT - LIFT_HEIGHT_THRESHOLD) || hasLiftReachedTargetHeight) {
            liftMotor.setTargetPosition(DOWN_HEIGHT);
            hasLiftReachedTargetHeight = true;
            liftMotor.setPower(1.0);
        }
    }

    public void setLiftPosition() {

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
