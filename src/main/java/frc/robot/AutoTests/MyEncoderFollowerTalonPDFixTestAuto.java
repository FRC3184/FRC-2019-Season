package frc.robot.AutoTests;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class MyEncoderFollowerTalonPDFixTestAuto extends CommandGroup {

    public MyEncoderFollowerTalonPDFixTestAuto() {
        addSequential(new AutoDriveMyEncoderFollowerTalonPID(new AutonomousDriveTrainMyEncoderFollowerTalonPID(), "example"));
    }
}
