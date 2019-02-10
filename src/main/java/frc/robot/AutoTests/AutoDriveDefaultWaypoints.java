package frc.robot.AutoTests;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoDriveDefaultWaypoints extends Command {
    AutonomousDriveTrainDefaultWaypoints drive;

    boolean firstRun;

    public AutoDriveDefaultWaypoints(AutonomousDriveTrainDefaultWaypoints drive) {
        requires(drive);
        this.drive = drive;
        firstRun = true;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        if (drive.gyroCalibrated()) {
            if (firstRun == false) {
                drive.followPath();
            } else {
                drive.setupPath();

                firstRun = false;
            }
        }

        SmartDashboard.putBoolean("Gyro Calibrated", drive.gyroCalibrated());
        SmartDashboard.putBoolean("finished", drive.pathCompete());
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
