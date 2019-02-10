package frc.robot.AutoTests;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.AutoTests.AutonomousDriveTrainDefault;

public class DefaultTestAuto extends CommandGroup {

    public DefaultTestAuto() {
        addSequential(new AutoDriveDefault(new AutonomousDriveTrainDefault(), "example"));
    }
}
