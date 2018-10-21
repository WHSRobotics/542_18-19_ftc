package org.whitneyrobotics.ftc.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.whitneyrobotics.ftc.lib.subsys.MotorSubsystem;

import java.util.Map;

public class Lift implements MotorSubsystem{

    private DcMotor liftMotor;
    private final int LIFT_HEIGHT = 3600;
    private final int FINAL_HEIGHT = 5376;
    private final int DOWN_HEIGHT = 0;
    private final int LIFT_HEIGHT_THRESHOLD=400;
    boolean b=false;

    public Lift(HardwareMap liftMap) {
        liftMotor = liftMap.dcMotor.get("liftMotor");
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setLiftMotorPower(double power){
        liftMotor.setPower(power);
    }

    public void liftUpRobot(){
        if(!b) {
            liftMotor.setTargetPosition(LIFT_HEIGHT);
            liftMotor.setPower(1.000000);
        }
        if (liftMotor.getTargetPosition()>(LIFT_HEIGHT-LIFT_HEIGHT_THRESHOLD) || b){
            liftMotor.setTargetPosition(DOWN_HEIGHT);
            b=true;
            liftMotor.setPower(1);
        }
    }

    public void extend(){

            liftMotor.setTargetPosition(FINAL_HEIGHT);
            liftMotor.setPower(.5);

    }

    public void  retract(){
        liftMotor.setTargetPosition(DOWN_HEIGHT);
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
