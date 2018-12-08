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

    public static double DEADBAND_DRIVE_TO_TARGET = 70;
    public static double DEADBAND_ROTATE_TO_TARGET = 0.5;
    public static double drive_min = 0.1;
    public static double drive_max = 1;
    public static double rotate_min = 0.2;
    public static double rotate_max = 1;
    public static double KP = 1;
    public static double KI = 0.8;
    public static double KD = 0.1;

    public static double rotateTestAngle = 90;
    public static boolean rotateOrientation = true;
}
