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
import frc.robot.RobotMap;
import frc.robot.commands.HatchHolder;

import javax.naming.ldap.Control;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class TeleOpHatch extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  TalonSRX motor;

  public TeleOpHatch () {
    motor = new TalonSRX(RobotMap.hatchIntake);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void grabHatch (int target) {
    if (motor.getSelectedSensorPosition() < target + 10) {
      motor.set(ControlMode.PercentOutput, .5);
    } else if (motor.getSelectedSensorPosition() > target - 10) {
      motor.set(ControlMode.PercentOutput, -.5);
    } else {
      motor.set(ControlMode.PercentOutput, 0);
    }
  }

  public void placeHatch (int target) {
    if (motor.getSelectedSensorPosition() < target + 10) {
      motor.set(ControlMode.PercentOutput, .5);
    } else if (motor.getSelectedSensorPosition() > target - 10) {
      motor.set(ControlMode.PercentOutput, -.5);
    } else {
      motor.set(ControlMode.PercentOutput, 0);
    }
  }
}
