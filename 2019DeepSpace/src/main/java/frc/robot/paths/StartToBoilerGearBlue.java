package frc.robot.paths;


import frc.robot.paths.profiles.PathAdapter;
import frc.lib.util.control.Path;
import frc.lib.util.math.RigidTransform2d;

/**
 * Path from the blue alliance wall to the blue boiler peg.
 * 
 * Used in GearThenHopperShootModeBlue
 * 
 * @see GearThenHopperShootModeBlue
 * @see PathContainer
 */
public class StartToBoilerGearBlue implements PathContainer {

    @Override
    public Path buildPath() {
        return PathAdapter.getBlueGearPath();
    }

    @Override
    public RigidTransform2d getStartPose() {
        return PathAdapter.getBlueStartPose();
    }

    @Override
    public boolean isReversed() {
        return true;
    }
}