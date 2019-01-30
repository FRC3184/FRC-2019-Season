package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.subsystems.AutonomousDriveTrainMyEncoderFollowerTalonPID;

public class MyEncoderFollowerTalonPDFixTestAuto extends CommandGroup {

    public MyEncoderFollowerTalonPDFixTestAuto() {
        addSequential(new AutoDriveMyEncoderFollowerTalonPID(new AutonomousDriveTrainMyEncoderFollowerTalonPID(), "example"));
    }
}
