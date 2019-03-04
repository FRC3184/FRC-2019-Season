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
        elevatorMaster = new TalonSRX(RobotMap.elevatorMaster);
        elevatorSlave = new TalonSRX(RobotMap.elevatorSlave);

        elevatorMaster.configFactoryDefault();
        elevatorSlave.configFactoryDefault();

        elevatorSlave.follow(elevatorMaster);

        elevatorMaster.getSelectedSensorPosition(0);

        elevatorMaster.config_kP(0, .00001);
        elevatorMaster.config_kI(0, 0);
        elevatorMaster.config_kD(0, 0);
        elevatorMaster.config_kF(0, 0);
        elevatorMaster.configClosedloopRamp(.33);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new TeleopDrive());
    }

    public void elevatorMoveToInches(int target) {
        double targetTicks = (target*countsPerInch);

        elevatorMaster.set(ControlMode.Position, targetTicks);
    }

    public void test(double power) {
        elevatorMaster.set(ControlMode.PercentOutput, power);
    }
}
