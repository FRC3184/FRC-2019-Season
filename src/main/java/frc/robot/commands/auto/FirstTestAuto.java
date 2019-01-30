package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.subsystems.AutonomousDriveTrain;

public class FirstTestAuto extends CommandGroup {

    public FirstTestAuto() {
        addSequential(new AutoDrive(new AutonomousDriveTrain(), "example"));
    }
}
