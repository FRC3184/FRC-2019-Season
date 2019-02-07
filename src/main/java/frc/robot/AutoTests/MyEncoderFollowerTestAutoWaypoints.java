package frc.robot.AutoTests;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class MyEncoderFollowerTestAutoWaypoints extends CommandGroup {

    public MyEncoderFollowerTestAutoWaypoints() {
        addSequential(new AutoDriveMyEncoderFollowerWaypoints(new AutonomousDriveTrainMyEncoderFollowerWaypoints()));
    }
}
