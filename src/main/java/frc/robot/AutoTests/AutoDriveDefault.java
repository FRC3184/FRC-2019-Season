package frc.robot.AutoTests;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AutoTests.AutonomousDriveTrainDefault;

public class AutoDriveDefault extends Command {
    AutonomousDriveTrainDefault drive;

    boolean firstRun;

    private static String m_path;

    public AutoDriveDefault(AutonomousDriveTrainDefault drive, String m_path) {
        requires(drive);
        this.drive = drive;
        this.m_path = m_path;
        firstRun = true;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        m_path = "example";
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        if (drive.gyroCalibrated()) {
            if (firstRun == false) {
                drive.followPath();
            } else {
                drive.setupPath(m_path);

                firstRun = false;
            }
        }

        SmartDashboard.putBoolean("Gyro Calibrated", drive.gyroCalibrated());
        SmartDashboard.putBoolean("finished", drive.pathComplete());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
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
