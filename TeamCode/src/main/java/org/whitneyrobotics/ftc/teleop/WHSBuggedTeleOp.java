package org.whitneyrobotics.ftc.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.lib.util.RobotConstants;
import org.whitneyrobotics.ftc.subsys.OmniArm;
import org.whitneyrobotics.ftc.subsys.WHSRobotImpl;

@TeleOp(name="BuggedTeleOp", group="tele")
public class WHSBuggedTeleOp extends OpMode{

    WHSRobotImpl robot;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    private double leftMultiplier = 1, rightMultiplier = 1;
    private double totalMultiplier = 1;
    private int canDrive = 1;
    private int canIntake = 1;
    private int canExtend = 1;
    private int canStoreArm = 1;
    private int canLift = 1;

    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
    }

    @Override
    public void loop() {
        /* Update variables based on Dashboard */
        leftMultiplier = RobotConstants.leftMultiplier;
        rightMultiplier = RobotConstants.rightMultiplier;
        totalMultiplier = RobotConstants.totalMultiplier;
        canDrive = RobotConstants.canDrive;
        canIntake = RobotConstants.canIntake;
        canExtend = RobotConstants.canExtend;
        canStoreArm = RobotConstants.canStoreArm;
        canLift = RobotConstants.canLift;

        packet.fieldOverlay()
                .setStroke("blue")
                .strokeRect(robot.getCoordinate().getX()/100/2.54, robot.getCoordinate().getY()/100/2.54, 20, 20);
        packet.put("Switch Current Pos ", robot.omniArm.pivotMotor.getCurrentPosition());
        packet.put("Switch Target Pos ", robot.omniArm.pivotMotor.getTargetPosition());
        //packet.put("Distance Sensor Distance ", robot.lift.distancer.getDistance(DistanceUnit.MM));
        packet.put("Coordinates", robot.getCoordinate());
        packet.put("Sensor Lift", gamepad1.y);
        dashboard.sendTelemetryPacket(packet);

        /* DRIVETRAIN */
        //Precision driving mode
        if (canDrive == 1) {
            if (gamepad1.left_bumper || gamepad2.b) {
                robot.drivetrain.operateWithOrientation(gamepad1.left_stick_y / 2.54 * leftMultiplier * totalMultiplier, gamepad1.right_stick_y / 2.54 * rightMultiplier * totalMultiplier);
            } else {
                robot.drivetrain.operateWithOrientation(gamepad1.left_stick_y * leftMultiplier * totalMultiplier, gamepad1.right_stick_y * rightMultiplier * totalMultiplier);
            }
            robot.drivetrain.switchOrientation(gamepad1.a);
        }

        if (canIntake == 1) {
            robot.omniArm.operateIntake(gamepad2.right_bumper, gamepad2.left_bumper, gamepad2.left_trigger>0.01);
        }
        if (canExtend == 1) {
            robot.omniArm.operateExtend(gamepad2.a);
        }
        if (canLift == 1) {
          //  robot.lift.sensorLift(gamepad1.y);
        }
        if (canStoreArm == 1) {
            if (gamepad1.right_bumper) {
                //robot.omniArm.setPivotPosition(OmniArm.PivotPosition.ROOM_FOR_LIFT);

            } else {
                robot.omniArm.operatePivot(gamepad2.x, gamepad2.y);
            }
        }

        if (canLift == 1) {
            if (gamepad2.dpad_down) {
                if (gamepad1.dpad_up) {
                    robot.lift.setLiftMotorPower(0.6);
                } else if (gamepad1.dpad_down) {
                    robot.lift.setLiftMotorPower(-0.66);
                } else {
                    robot.lift.setLiftMotorPower(0.0);
                }
            } else {
                robot.lift.liftUpRobot(gamepad2.dpad_up);
            }
        }
    }
}
