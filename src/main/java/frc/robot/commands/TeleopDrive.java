/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import jaci.pathfinder.Pathfinder;

/**
 * An example command.  You can replace me with your own command.
 */
public class TeleopDrive extends Command {
  DriveTrain drive;
  AHRS nav_x;

  public TeleopDrive(DriveTrain drive) {
    // Use requires() here to declare subsystem dependencies
    requires(drive);
    this.drive = drive;

    nav_x = new AHRS(SPI.Port.kMXP);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    drive.leftMaster.setSelectedSensorPosition(0);
    drive.rightMaster.setSelectedSensorPosition(0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    drive.arcadeDrive(-OI.get().getPower(), OI.get().getTurn());

    SmartDashboard.putNumber("Left Motor Encoder", drive.leftMaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Motor Encoder", drive.rightMaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("Gyro Raw Yaw", nav_x.getYaw());
    SmartDashboard.putNumber("Gyro to radians", Pathfinder.r2d(nav_x.getYaw()));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
