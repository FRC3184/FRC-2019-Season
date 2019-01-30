package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.subsystems.AutonomousDriveTrainMyEncoderFollower;

public class MyEncoderFollowerTestAuto extends CommandGroup {

    public MyEncoderFollowerTestAuto() {
        addSequential(new AutoDriveMyEncoderFollower(new AutonomousDriveTrainMyEncoderFollower(), "example"));
    }
}
