package frc.robot.AutoTests;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class DefaultTestAutoWaypoints extends CommandGroup {

    public DefaultTestAutoWaypoints() {
        addSequential(new AutoDriveDefaultWaypoints(new AutonomousDriveTrainDefaultWaypoints()));
    }
}
