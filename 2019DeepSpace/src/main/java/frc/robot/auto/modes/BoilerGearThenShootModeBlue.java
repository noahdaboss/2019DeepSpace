package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.DrivePathAction;
import frc.robot.auto.actions.ResetPoseFromPathAction;
import frc.robot.auto.actions.WaitAction;
import frc.robot.paths.BoilerGearToShootBlue;
import frc.robot.paths.PathContainer;
import frc.robot.paths.StartToBoilerGearBlue;

/**
 * Scores the preload gear onto the boiler-side peg then shoots the 10 preloaded fuel
 * 
 * @see AutoModeBase
 */
public class BoilerGearThenShootModeBlue extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new WaitAction(2));
        PathContainer gearPath = new StartToBoilerGearBlue();
        runAction(new ResetPoseFromPathAction(gearPath));
        runAction(new DrivePathAction(gearPath));       
        runAction(new DrivePathAction(new BoilerGearToShootBlue()));             
    }
}
