/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class TeleOpDriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public WPI_TalonSRX leftMaster;
  public WPI_TalonSRX rightMaster;
  VictorSPX leftSlave;
  VictorSPX rightSlave;

  DifferentialDrive robotDrive;

  public TeleOpDriveTrain() {
    leftMaster = new WPI_TalonSRX(RobotMap.leftDriveMaster);
    rightMaster = new WPI_TalonSRX(RobotMap.rightDriveMaster);
    leftSlave = new VictorSPX(RobotMap.leftDriveSlave);
    rightSlave = new VictorSPX(RobotMap.rightDriveSlave);

    robotDrive = new DifferentialDrive(leftMaster, rightMaster);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    //setDefaultCommand(new TeleopDrive());
  }

  public void arcadeDrive(double power, double turn) {
    robotDrive.arcadeDrive(power, turn);
  }

  public void tankDrive(double leftPower, double rightPower) {
    robotDrive.tankDrive(leftPower, rightPower);
  }
}
