package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.subsystems.AutonomousDriveTrainMyEncoderFollowerTalonPID;

public class MyEncoderFollowerTalonPIDTestAuto extends CommandGroup {

    public MyEncoderFollowerTalonPIDTestAuto() {
        addSequential(new AutoDriveMyEncoderFollowerTalonPID(new AutonomousDriveTrainMyEncoderFollowerTalonPID(), "example"));
    }
}
