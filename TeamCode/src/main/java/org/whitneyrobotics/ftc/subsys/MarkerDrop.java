package org.whitneyrobotics.ftc.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MarkerDrop {
    private Servo x;
    private static final double DOWN_POSITION = -900;
    private static final double UP_POSITION = 0;

    public MarkerDrop(HardwareMap markerMap){
        x = markerMap.servo.get("marker");
    }

    public void operateMarkerDrop(boolean b){
        if (b) {
            x.setPosition(DOWN_POSITION);
            b=false;
        }else {
            x.setPosition(UP_POSITION);
        }
    }
}
