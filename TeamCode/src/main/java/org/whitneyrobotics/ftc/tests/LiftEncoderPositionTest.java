package org.whitneyrobotics.ftc.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.whitneyrobotics.ftc.lib.util.Toggler;
import org.whitneyrobotics.ftc.subsys.Lift;
import org.whitneyrobotics.ftc.subsys.WHSRobotImpl;

@TeleOp(name = "LiftTest and OmniArmTest", group = "tests")
public class LiftEncoderPositionTest extends OpMode {

    DcMotor lift;
    DcMotor switchMotor;
    DcMotor extendMotor;
    DcMotor intakeMotor;
    Toggler xtog = new Toggler(2);
    Toggler ytog = new Toggler(2);
    int encoderPos = 0;
   // WHSRobotImpl robot;
    Toggler runToPositionTog = new Toggler(2);

    @Override
    public void init() {
      //  robot = new WHSRobotImpl(hardwareMap);
        lift = hardwareMap.dcMotor.get("liftMotor");
        switchMotor = hardwareMap.dcMotor.get("switchMotor");
        extendMotor = hardwareMap.dcMotor.get("extendMotor");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setPower(0.0);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

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
            lift.setPower(.666);
        }else if (gamepad1.y){
            lift.setPower(-.666);
        }else if (gamepad1.right_bumper){
            switchMotor.setPower(.666);
        }
        else if(gamepad1.left_bumper){
            switchMotor.setPower(-.666);
        }
        else if(gamepad1.right_trigger > .01){
            intakeMotor.setPower(.5);
        }
        else if(gamepad1.left_trigger > .01){
            intakeMotor.setPower(-.5);
        }
        else if(gamepad1.dpad_up){
            extendMotor.setPower(.5);
        }else if(gamepad1.dpad_down){
            extendMotor.setPower(-.5);
        }
        else{
            lift.setPower(0);
            switchMotor.setPower(0);
            extendMotor.setPower(0);
            intakeMotor.setPower(0);
        }


      /*  if(gamepad1.x){
            if (xtog.currentState() == 0) {
                lift.setPower(.5);
                xtog.changeState(true);
            }else{
                lift.setPower(0);
                xtog.changeState(true);
            }

        }else if (gamepad1.y) {
            if (ytog.currentState() == 0) {
                lift.setPower(-.5);
                ytog.changeState(true);
            }else{
                lift.setPower(0);
                ytog.changeState(true);
            }
        }*/
         if(gamepad1.right_bumper) {
            lift.setPower(1.0);
        }/*else if(gamepad1.right_trigger > 0.10) {
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
*/

        telemetry.addData("Motor Target Pos", encoderPos);
        telemetry.addData("Motor Current Pos", lift.getCurrentPosition());
    }
}
