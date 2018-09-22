package org.whitneyrobotics.ftc.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.whitneyrobotics.ftc.subsys.IMU;

/**
 * Created by ivanm on 11/4/2017.
 */
@Autonomous(name = "IMUTest", group = "tests")
public class IMUTest extends OpMode {

    IMU imu;

    @Override
    public void init() {
        imu = new IMU(hardwareMap);
    }

    @Override
    public void loop() {

        double heading = imu.getHeading();

        double[] threeHeading = imu.getThreeHeading();



        telemetry.addData("Heading: ", heading);

       /* if(gamepad1.x){
            imu.();
        }*/

        telemetry.addData("x", threeHeading[0]);
        telemetry.addData("y", threeHeading[1]);
        telemetry.addData("z", threeHeading[2]);

    }
}
