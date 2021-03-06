package org.whitneyrobotics.ftc.lib.util;

/**
 * General purpose functions class
 */
public class Functions
{
    public static double calculateDistance(Position current, Position target)
    {
        double distance;
        distance = Math.sqrt(Math.pow(target.getX() - current.getX(), 2) +
                    Math.pow(target.getY() - current.getY(), 2));
        return distance;
    }

    /** Converts angles from 0-360 to -180-180 */
    public static double normalizeAngle(double angle){

        if(angle>180){
            angle=angle-360;
        }
        else if(angle<-180){
            angle=angle+360;
        }
        /*
        else {
            angle=angle;
        }
        */
        return angle;

    }

    public static Position transformCoordinates(double[][] dcm, Position vector)
    {
        Position transformedVector;

        double x = dcm[0][0]*vector.getX() + dcm[0][1]*vector.getY() + dcm[0][2]*vector.getZ();
        double y = dcm[1][0]*vector.getX() + dcm[1][1]*vector.getY() + dcm[1][2]*vector.getZ();
        double z = dcm[2][0]*vector.getX() + dcm[2][1]*vector.getY() + dcm[2][2]*vector.getZ();

        transformedVector = new Position(x,y,z);
        return transformedVector;
    }

    public static double cosd(double degree)
    {
        double rad = degree * Math.PI / 180;
        return Math.cos(rad);
    }

    public static double sind(double degree)
    {
        double rad = degree * Math.PI / 180;
        return Math.sin(rad);
    }

    public static Position addPositions(Position pos1, Position pos2)
    {
        Position sum;

        double x = pos1.getX() + pos2.getX();
        double y = pos1.getY() + pos2.getY();
        double z = pos1.getZ() + pos2.getZ();

        sum = new Position(x,y,z);
        return sum;
    }

    public static Position subtractPositions(Position pos1, Position pos2)
    {
        Position difference;

        double x = pos1.getX() - pos2.getX();
        double y = pos1.getY() - pos2.getY();
        double z = pos1.getZ() - pos2.getZ();

        difference = new Position(x,y,z);
        return difference;
    }

    public static double calculateMagnitude(Position pos)
    {
        double magnitude = Math.pow(pos.getX(), 2) + Math.pow(pos.getY(), 2);
        magnitude = Math.sqrt(magnitude);
        return magnitude;
    }

    public static double map(double x, double in_min, double in_max, double out_min, double out_max)
    {
        if (x <= in_min) {
            return out_min;
        }
        if (x >= in_max) {
            return out_max;
        }
        if (in_min >= in_max) {
            throw new IllegalArgumentException("in_min greater than in_max");
        }
        if (out_min >= out_max) {
            throw new IllegalArgumentException("out_min greater than out_max");
        }
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}
