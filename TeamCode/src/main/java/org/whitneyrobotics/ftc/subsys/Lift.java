package org.whitneyrobotics.ftc.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Map;

public class Lift {

    public DcMotor liftMotor;
    private final int LIFT_HEIGHT = 0;
    private final int FINAL_HEIGHT = 0;
    private final int DOWN_HEIGHT = 0;

    public Lift(HardwareMap liftMap) {
        liftMotor = liftMap.dcMotor.get("LiftMotor");
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void liftUpRobot(){
     liftMotor.setTargetPosition(LIFT_HEIGHT);
     liftMotor.setTargetPosition(DOWN_HEIGHT);
    }

    public void extend(){

            liftMotor.setTargetPosition(FINAL_HEIGHT);

    }

    public void  retract(){
        liftMotor.setTargetPosition(DOWN_HEIGHT);
    }
}
