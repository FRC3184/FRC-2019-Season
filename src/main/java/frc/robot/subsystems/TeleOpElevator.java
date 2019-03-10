/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class TeleOpElevator extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public TalonSRX elevatorMaster;
    public TalonSRX elevatorSlave;
    SensorCollection elevatorLimitSwitches;

    static final double countsPerOSRev = 4096;
    static final double sprocketPitchDiameter = 4; //IN INCHES
    static final double cascadeOffset = 5;
    static final double topInches = 50;
    static final double countsPerInchBeforeCascade = (countsPerOSRev/sprocketPitchDiameter);
    static final double countsPerInchAfterCascade = (countsPerOSRev/(sprocketPitchDiameter * 2));

    public TeleOpElevator() {
        elevatorMaster = new TalonSRX(RobotMap.elevatorMaster);
        elevatorSlave = new TalonSRX(RobotMap.elevatorSlave);

        elevatorLimitSwitches = elevatorMaster.getSensorCollection();

        elevatorMaster.configFactoryDefault();
        elevatorSlave.configFactoryDefault();

        elevatorSlave.follow(elevatorMaster);

        elevatorMaster.setSelectedSensorPosition(0);

        elevatorMaster.setSensorPhase(true);

        elevatorMaster.config_kP(0, .25);
        elevatorMaster.config_kI(0, 0);
        elevatorMaster.config_kD(0, 10.0);
        elevatorMaster.config_kF(0, 0.005);
        elevatorMaster.configClosedloopRamp(.33);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new DriveTest());
    }

    public void elevatorMoveToInches(double target) {
        double targetTicks = -inchesToTicks(target);

        elevatorMaster.set(ControlMode.Position, targetTicks);
    }

    public void testSwitches() {
        if (elevatorLimitSwitches.isFwdLimitSwitchClosed()) {
            elevatorMaster.setSelectedSensorPosition(0);
        } else if (elevatorLimitSwitches.isRevLimitSwitchClosed()) {
            elevatorMaster.setSelectedSensorPosition(inchesToTicks(topInches));
        }
    }

    int inchesToTicks(double inches) {
        double ticks;

        if (inches <= cascadeOffset) {
            ticks = inches * countsPerInchBeforeCascade;
        } else {
            ticks = ((inches - cascadeOffset) * countsPerInchAfterCascade) + (cascadeOffset * countsPerInchBeforeCascade);
        }

        return (int)ticks;
    }

    public void test(double power) {
        elevatorMaster.set(ControlMode.PercentOutput, power);
    }
}