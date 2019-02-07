package frc.robot.AutoTests;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoDriveMyEncoderFollowerTalonPIDFixWaypoints extends Command {
    AutonomousDriveTrainMyEncoderFollowerTalonPIDFixWaypoints drive;

    public AutoDriveMyEncoderFollowerTalonPIDFixWaypoints(AutonomousDriveTrainMyEncoderFollowerTalonPIDFixWaypoints drive) {
        requires(drive);
        this.drive = drive;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        if (drive.gyroCalibrated()) {
            drive.followPath();
        }

        SmartDashboard.putBoolean("Gtro Calibrted", drive.gyroCalibrated());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return drive.pathCompete();
    }

    // Called once after isFinished returns true
    protected void end() {
        drive.letGo();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
}
