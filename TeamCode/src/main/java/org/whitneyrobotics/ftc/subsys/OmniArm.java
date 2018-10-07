package org.whitneyrobotics.ftc.subsys;

import android.print.PageRange;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OmniArm {

    public DcMotor extendMotor;
    public DcMotor intakeMotor;
    public DcMotor swtichMotor;

    private final int EXTEND_LENGTH = 0;
    private final int CONTRACT_LENGTH = 0;
    private final int INTAKE_MODE = 0;
    private final int OUTTAKE_MODE = 0;
    private final int INTAKE_SPEED = 0;
    private final int OUTTAKE_SPEED = 0;



    public OmniArm(HardwareMap armMap) {
        extendMotor = armMap.dcMotor.get("extendMotor");
        intakeMotor = armMap.dcMotor.get("intakeMotor");
        swtichMotor = armMap.dcMotor.get("switchMotor");
        extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        swtichMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

    public void changemode(){
        if(swtichMotor.getTargetPosition()== INTAKE_MODE){

            swtichMotor.setTargetPosition(OUTTAKE_MODE);

        }else if (swtichMotor.getTargetPosition()==OUTTAKE_MODE){

            swtichMotor.setTargetPosition(INTAKE_MODE);

        }
    }



}
