package org.whitneyrobotics.ftc.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.whitneyrobotics.ftc.lib.util.Toggler;
import org.whitneyrobotics.ftc.subsys.Lift;
import org.whitneyrobotics.ftc.subsys.WHSRobotImpl;

@TeleOp(name = "LiftTest", group = "tests")
public class LiftEncoderPositionTest extends OpMode {

    DcMotor lift;
    int encoderPos = 0;
    WHSRobotImpl robot;
    Toggler runToPositionTog = new Toggler(2);

    @Override
    public void init() {
        lift = hardwareMap.dcMotor.get("liftMotor");
    }

    @Override
    public void loop() {

        runToPositionTog.changeState(gamepad1.left_bumper);

        if(gamepad1.a ){
            encoderPos++;
        }
        else if(gamepad1.b){
            encoderPos--;
        }
        if(gamepad1.x){
            robot.lift.setLiftMotorPower(.666);
        }else if (gamepad1.y) {
            robot.lift.setLiftMotorPower(-.666);
        }
        else if(gamepad1.right_bumper) {
            lift.setPower(1.0);
        }else if(gamepad1.right_trigger > 0.10) {
            robot.lift.setLiftMotorPower(-1.0);

        }else if (gamepad1.dpad_up){
            robot.lift.liftUpRobot();
        }else{
            lift.setPower(0);
        }

        if(runToPositionTog.currentState() == 0){
            robot.lift.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if(runToPositionTog.currentState() == 1){
            robot.lift.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        }


        lift.setTargetPosition(encoderPos);
        telemetry.addData("Motor Current Pos", encoderPos);
        telemetry.addData("Motor Target Pos", lift.getCurrentPosition());
    }
}
