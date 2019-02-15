/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class TeleOpElevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public TalonSRX elevatorMaster;
  public TalonSRX elevatorSlave;

  DifferentialDrive robotDrive;

  public TeleOpElevator() {
    elevatorMaster = new TalonSRX(RobotMap.leftDriveMaster);
    elevatorSlave = new TalonSRX(RobotMap.rightDriveMaster);

    elevatorSlave.follow(elevatorMaster);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    //setDefaultCommand(new TeleopDrive());
  }

  public void elevatorPreset1(int target) {
    if (elevatorMaster.getSelectedSensorPosition()<target+10 ){elevatorMaster.set(ControlMode.PercentOutput, .25);}
    else{elevatorMaster.set(ControlMode.PercentOutput, 0);
    }
  }
  public void elevatorPreset0(int target) {
    if (elevatorMaster.getSelectedSensorPosition()>target-10 ){elevatorMaster.set(ControlMode.PercentOutput, -0.25);}
    else{elevatorMaster.set(ControlMode.PercentOutput, 0);
    }
  }
  public void elevatorPresetGeneral(int target) {
    if (elevatorMaster.getSelectedSensorPosition()<target ){elevatorMaster.set(ControlMode.PercentOutput, .25);}
    else if (elevatorMaster.getSelectedSensorPosition()>target ){elevatorMaster.set(ControlMode.PercentOutput, -.25);}
    else{elevatorMaster.set(ControlMode.PercentOutput, 0);
    }
  }

  public void tankDrive(double leftPower, double rightPower) {
    robotDrive.tankDrive(-leftPower, -rightPower);
  }
}
