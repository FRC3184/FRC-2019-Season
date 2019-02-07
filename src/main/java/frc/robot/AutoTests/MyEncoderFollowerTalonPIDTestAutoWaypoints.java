package frc.robot.AutoTests;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class MyEncoderFollowerTalonPIDTestAutoWaypoints extends CommandGroup {

    public MyEncoderFollowerTalonPIDTestAutoWaypoints() {
        addSequential(new AutoDriveMyEncoderFollowerTalonPIDWaypoints(new AutonomousDriveTrainMyEncoderFollowerTalonPIDWaypoints()));
    }
}
