package frc.robot.AutoTests;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class MyEncoderFollowerTalonPDFixTestAutoWaypoints extends CommandGroup {

    public MyEncoderFollowerTalonPDFixTestAutoWaypoints() {
        addSequential(new AutoDriveMyEncoderFollowerTalonPIDWaypoints(new AutonomousDriveTrainMyEncoderFollowerTalonPIDWaypoints()));
    }
}
