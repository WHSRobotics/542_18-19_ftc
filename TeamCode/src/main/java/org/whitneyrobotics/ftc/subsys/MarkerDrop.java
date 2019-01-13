package org.whitneyrobotics.ftc.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MarkerDrop {
    private Servo markerDropServo;
    private static final double[] MARKER_DROP_POSITIONS = {0.31,0.09}; //Stored, Dumped
    public enum MarkerDropPosition {
        STORED, DUMPED
    }

    public MarkerDrop(HardwareMap markerMap){
        markerDropServo = markerMap.servo.get("marker");
    }

    public void operateMarkerDrop(double position){
        markerDropServo.setPosition(position);
    }

    public void operateMarkerDrop(MarkerDropPosition markerDropPosition) {
        markerDropServo.setPosition(MARKER_DROP_POSITIONS[markerDropPosition.ordinal()]);
    }
}
