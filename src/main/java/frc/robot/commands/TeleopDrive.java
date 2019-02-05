/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.subsystems.TeleOpDriveTrain;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.opencv.core.Core;

/**
 * An example command.  You can replace me with your own command.
 */
public class TeleopDrive extends Command {
  TeleOpDriveTrain drive;
  AHRS nav_x;

  double KpAim;
  double KpDistance;
  double min_aim_command;
  double heading_error;
  double distance_error;
  double steering_adjust;
  double x;
  double y;
  double area;
  double leftPower;
  double rightPower;
  double distance_adjust;

  public TeleopDrive(TeleOpDriveTrain drive) {
    // Use requires() here to declare subsystem dependencies
    requires(drive);
    this.drive = drive;

    nav_x = new AHRS(SPI.Port.kMXP);

    System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
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
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("Left Power", leftPower);
    SmartDashboard.putNumber("Right Power", rightPower);

    if (OI.get().getAlign()) {
      float Kp = -0.01f;
      float min_command = 0.3f;

      double heading_error = -x;
      double steering_adjust = 0.0f;

      if (x > .25f) {
        steering_adjust = Kp*heading_error + min_command;
      }
      else if (x < -.25f) {
        steering_adjust = Kp*heading_error - min_command;
      }

      leftPower = +steering_adjust;
      rightPower = -steering_adjust;

      drive.tankDrive(leftPower, rightPower);
    } else {
      drive.arcadeDrive(OI.get().getPower(), OI.get().getTurn());
    }
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
