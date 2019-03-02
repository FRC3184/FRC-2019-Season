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

    static final double countsPerOSRev = 4096;
    static final double sprocketPitchDiameter = 1.282; //IN INCHES
    static final double countsPerInch = (countsPerOSRev/sprocketPitchDiameter);

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

    public void elevatorMoveToInches(int target) {
        double targetTicks = (target*countsPerInch);

        if (elevatorMaster.getSelectedSensorPosition() < targetTicks ){
            elevatorMaster.set(ControlMode.PercentOutput, .25);
        } else if (elevatorMaster.getSelectedSensorPosition() > targetTicks ){
            elevatorMaster.set(ControlMode.PercentOutput, -.25);
        } else{
            elevatorMaster.set(ControlMode.PercentOutput, 0);
        }
    }
}
