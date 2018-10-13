package org.whitneyrobotics.ftc.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OmniArm {

    public DcMotor extendMotor;
    public DcMotor intakeMotor;
    public DcMotor switchMotor;

    private final int EXTEND_LENGTH = 0;
    private final int CONTRACT_LENGTH = 0;
    private final int INTAKE_MODE = 0;
    private final int OUTTAKE_MODE = 0;
    private final int INTAKE_SPEED = 0;
    private final int OUTTAKE_SPEED = 0;



    public OmniArm(HardwareMap armMap) {
        extendMotor = armMap.dcMotor.get("extendMotor");
        intakeMotor = armMap.dcMotor.get("intakeMotor");
        switchMotor = armMap.dcMotor.get("switchMotor");
        extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        switchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void  intake(){
        intakeMotor.setPower(INTAKE_SPEED);
    }

    public void outtake(){
        intakeMotor.setPower(OUTTAKE_SPEED);
    }

    public void extendIntake(){
        intakeMotor.setTargetPosition(EXTEND_LENGTH);
    }

    public void contractIntake(){
        intakeMotor.setTargetPosition(CONTRACT_LENGTH);
    }

    public void changeMode(){
        if(switchMotor.getTargetPosition()== INTAKE_MODE){

            switchMotor.setTargetPosition(OUTTAKE_MODE);

        }else if (switchMotor.getTargetPosition()==OUTTAKE_MODE){

            switchMotor.setTargetPosition(INTAKE_MODE);

        }
    }



}
