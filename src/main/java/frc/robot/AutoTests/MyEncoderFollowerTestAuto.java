package frc.robot.AutoTests;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class MyEncoderFollowerTestAuto extends CommandGroup {

    public MyEncoderFollowerTestAuto() {
        addSequential(new AutoDriveMyEncoderFollower(new AutonomousDriveTrainMyEncoderFollower(), "example"));
    }
}
