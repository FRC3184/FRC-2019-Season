package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.subsystems.AutonomousDriveTrainDefault;

public class DefaultTestAuto extends CommandGroup {

    public DefaultTestAuto() {
        addSequential(new AutoDriveDefault(new AutonomousDriveTrainDefault(), "example"));
    }
}
