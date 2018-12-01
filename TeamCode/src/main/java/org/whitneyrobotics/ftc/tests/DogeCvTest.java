package org.whitneyrobotics.ftc.tests;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.whitneyrobotics.ftc.lib.util.Coordinate;
import org.whitneyrobotics.ftc.lib.util.Position;
import org.whitneyrobotics.ftc.subsys.WHSRobotImpl;

import org.whitneyrobotics.ftc.subsys.WHSRobotImpl;

@Autonomous(name="DogeCVTest", group="auto")

public class DogeCvTest extends OpMode {

    private GoldAlignDetector detector;

    double Xpos;

    public void init(){

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext,CameraViewDisplay.getInstance());
        detector.useDefaults();

        detector.enable();

    }

    @Override
    public void loop() {
        Xpos = detector.getXPosition();
        if( Xpos < 200){

            telemetry.addData("Position", "left");

        }else if (Xpos > 200 && Xpos <400){

            telemetry.addData("Position", "Center");

        }else if (Xpos >400){

            telemetry.addData("Position","Right" );

        }
        telemetry.addData("Xposition : ", Xpos);

    }
}
