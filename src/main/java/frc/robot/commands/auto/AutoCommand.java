package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.subsystems.AutonomousDriveTrain;

public class AutoCommand extends CommandGroup {

    public AutoCommand() {
        addSequential(new AutoDrive(new AutonomousDriveTrain(), "example"));
    }
}
