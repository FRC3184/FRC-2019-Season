package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.subsystems.AutonomousDriveTrain;

public class AutoCommand extends CommandGroup {

    public AutoCommand(String selected) {
        parseSelected(selected);

        addSequential(new AutoDrive(new AutonomousDriveTrain(), "example"));
    }

    public String parseSelected(String selected) {
        return "example";
    }
}
