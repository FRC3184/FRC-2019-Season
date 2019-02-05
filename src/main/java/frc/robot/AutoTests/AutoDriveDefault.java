package frc.robot.AutoTests;

import edu.wpi.first.wpilibj.command.Command;

public class AutoDriveDefault extends Command {
    AutonomousDriveTrainDefault drive;

    private static String m_path;

    public AutoDriveDefault(AutonomousDriveTrainDefault drive, String m_path) {
        requires(drive);
        this.drive = drive;
        this.m_path = m_path;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        m_path = "example";

        drive.setupPath(m_path);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        drive.followPath();
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
