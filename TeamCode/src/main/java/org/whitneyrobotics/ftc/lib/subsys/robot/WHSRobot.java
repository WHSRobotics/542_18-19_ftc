package org.whitneyrobotics.ftc.lib.subsys.robot;


import org.whitneyrobotics.ftc.lib.util.Coordinate;
import org.whitneyrobotics.ftc.lib.util.Position;

/**
 * Created by Jason on 10/20/2017.
 */

public interface WHSRobot {

    public void driveToTarget(Position targetPos, boolean backwards);

    public void rotateToTarget(double targetHeading, boolean backwards); //-180 to 180 degrees

    public boolean driveToTargetInProgress();

    public boolean rotateToTargetInProgress();

    public boolean hasDriveToTargetExited();

    public boolean hasRotateToTargetExited();

    public void estimatePosition();

    public void estimateHeading();

    public void setInitialCoordinate(Coordinate initCoord);

    public void setCoordinate(Coordinate coord);

    public Coordinate getCoordinate();

}
