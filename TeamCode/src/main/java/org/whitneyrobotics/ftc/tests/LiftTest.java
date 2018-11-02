package org.whitneyrobotics.ftc.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.lib.util.Toggler;
import org.whitneyrobotics.ftc.subsys.Lift;
@TeleOp(name = "LiftTest", group = "tests")
public class LiftTest extends OpMode {

    Lift lift;
    Toggler liftPositionTog = new Toggler(4);
    Toggler testModeTog = new Toggler(2);
    boolean hasLiftUpBeenPressed;

    @Override
    public void init() {
        lift = new Lift(hardwareMap);
    }

    @Override
    public void loop() {
        testModeTog.changeState(gamepad1.a);
        if(testModeTog.currentState() == 0){
            liftPositionTog.changeState(gamepad1.dpad_up, gamepad1.dpad_down);
            switch (liftPositionTog.currentState()){
                case 0: lift.setLiftPosition(Lift.LiftPosition.STORED);
                case 1: lift.setLiftPosition(Lift.LiftPosition.IN_LATCH);
                case 2: lift.setLiftPosition(Lift.LiftPosition.ABOVE_LATCH);
                case 3: lift.setLiftPosition(Lift.LiftPosition.FINAL);
            }
        }
        if(testModeTog.currentState() == 1){
            if((gamepad1.left_bumper && gamepad1.right_bumper) || hasLiftUpBeenPressed){
                lift.liftUpRobot();
                hasLiftUpBeenPressed = true;
            }
        }
        telemetry.addData("Mode", testModeTog.currentState()==0 ? "Incremental (dpad)" : "TeleOp Style");
        telemetry.addData("Pos", liftPositionTog.currentState());
    }
}
