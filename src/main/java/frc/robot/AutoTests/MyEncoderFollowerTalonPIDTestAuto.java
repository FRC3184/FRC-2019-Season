package frc.robot.AutoTests;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class MyEncoderFollowerTalonPIDTestAuto extends CommandGroup {

    public MyEncoderFollowerTalonPIDTestAuto() {
        addSequential(new AutoDriveMyEncoderFollowerTalonPID(new AutonomousDriveTrainMyEncoderFollowerTalonPID(), "example"));
    }
}
