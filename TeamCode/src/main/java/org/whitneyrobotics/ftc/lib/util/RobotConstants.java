package org.whitneyrobotics.ftc.lib.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    public static double leftMultiplier = 1;
    public static double rightMultiplier = 1;
    public static double totalMultiplier = 1/2.54;
    public static int canDrive = 1;
    public static int canIntake = 1;
    public static int canExtend = 1;
    public static int canStoreArm = 1;
    public static int canLift = 1;

    public static double DEADBAND_DRIVE_TO_TARGET = 24.5;
    public static double DEADBAND_ROTATE_TO_TARGET = 2;
    public static double drive_min = .1245;
    public static double drive_max = .6;
    public static double rotate_min = 0.2;
    public static double rotate_max = 1;
    public static double R_KP = 1.19;
    public static double R_KI = 0.85;
    public static double R_KD = 0.47;
    public static double D_KP = 1.5;
    public static double D_KI = 0.7;
    public static double D_KD = 0.8;

    public static double rotateTestAngle = 90;
    public static boolean rotateOrientation = true;
}
