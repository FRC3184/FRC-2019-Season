/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.test;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.subsystems.TeleOpDriveTrain;

/**
 * An example command.  You can replace me with your own command.
 */
public class DriveTest extends Command {
    TeleOpDriveTrain drive;
    AHRS nav_x;

    double KpAim;
    double KpDistance;
    double min_aim_command;
    double z;
    double x;
    double xDeg;
    double[] pos;
    boolean firstRun = true;

    public DriveTest(TeleOpDriveTrain drive) {
        // Use requires() here to declare subsystem dependencies
        requires(drive);
        this.drive = drive;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        KpAim = -0.1f;
        KpDistance = -0.1f;
        min_aim_command = 0.05f;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        drive.arcadeDrive(OI.get().testPower(), OI.get().testTurn());

        OI.get().updateLayerShift();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        drive.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}