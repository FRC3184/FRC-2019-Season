package frc.robot.AutoTests;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class DefaultTestAuto extends CommandGroup {

    public DefaultTestAuto() {
        addSequential(new AutoDriveDefault(new AutonomousDriveTrainDefault(), "example"));
    }
}
