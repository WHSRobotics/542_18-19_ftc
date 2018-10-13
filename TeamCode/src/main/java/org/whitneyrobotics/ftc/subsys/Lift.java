package org.whitneyrobotics.ftc.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Map;

public class Lift {

    public DcMotor liftMotor;
    private final int LIFT_HEIGHT = 0;
    private final int FINAL_HEIGHT = 0;
    private final int DOWN_HEIGHT = 0;
    private final int LIFT_HEIGHT_THRESHOLD=0;
    boolean b=false;

    public Lift(HardwareMap liftMap) {
        liftMotor = liftMap.dcMotor.get("liftMotor");
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void liftUpRobot(){
        if(!b) {
            liftMotor.setTargetPosition(LIFT_HEIGHT);
            liftMotor.setPower(1.000000);
        }
        if (liftMotor.getTargetPosition()>(LIFT_HEIGHT-LIFT_HEIGHT_THRESHOLD) || b){
            liftMotor.setTargetPosition(DOWN_HEIGHT);
            b=true;
            liftMotor.setPower(00000000001.000000000000);
        }
    }

    public void extend(){

            liftMotor.setTargetPosition(FINAL_HEIGHT);

    }

    public void  retract(){
        liftMotor.setTargetPosition(DOWN_HEIGHT);
    }
}
