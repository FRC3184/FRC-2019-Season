/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class TeleOpElevator extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public TalonSRX elevatorMaster;
    public TalonSRX elevatorSlave;
    public DigitalInput forwardLimitSwitch;
    public DigitalInput reverseLimitSwitch;

    public double targetT = 0;

    private static final double forwardMaxPower = .8; //Elevator up
    private double reverseMaxPower = -.4; //Elevator down
    private static final double countsPerOSRev = 4096;
    private static final double sprocketPitchDiameter = 4; //IN INCHES
    private static final double cascadeOffset = 5;
    private static final double topInches = 48.25;
    private static final double countsPerInchBeforeCascade = (countsPerOSRev/sprocketPitchDiameter);
    private static final double countsPerInchAfterCascade = (countsPerOSRev/(sprocketPitchDiameter * 2));

    public TeleOpElevator() {
        elevatorMaster = new TalonSRX(RobotMap.elevatorMaster);
        elevatorSlave = new TalonSRX(RobotMap.elevatorSlave);

        forwardLimitSwitch = new DigitalInput(RobotMap.elevatorLimitSwitchForward);
        reverseLimitSwitch = new DigitalInput(RobotMap.elevatorLimitSwitchReverse);

        elevatorMaster.configFactoryDefault();
        elevatorSlave.configFactoryDefault();

        elevatorSlave.follow(elevatorMaster);

        elevatorMaster.setSelectedSensorPosition(0);

        elevatorMaster.setSensorPhase(false);

        elevatorMaster.config_kP(0, 1.0);
        elevatorMaster.config_kI(0, 0);
        elevatorMaster.config_kD(0, 10.0);
        elevatorMaster.config_kF(0, 0);
        elevatorMaster.configClosedloopRamp(.33);

        elevatorMaster.configPeakOutputForward(forwardMaxPower);
        elevatorMaster.configPeakOutputReverse(reverseMaxPower);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new DriveTest());
    }

    public void elevatorMoveToInches(double target) {
        double targetTicks = inchesToTicks(target);

        targetT = target;

        elevatorMaster.set(ControlMode.Position, targetTicks);
    }

    public void testSwitches() {
        if (!forwardLimitSwitch.get()) {
            elevatorMaster.setSelectedSensorPosition(inchesToTicks(topInches));

            elevatorMaster.configPeakOutputForward(0);
            elevatorMaster.configPeakOutputReverse(reverseMaxPower);
        } else if (!reverseLimitSwitch.get()) {
            elevatorMaster.setSelectedSensorPosition(0);

            elevatorMaster.configPeakOutputForward(forwardMaxPower);
            elevatorMaster.configPeakOutputReverse(0);
        } else {
            elevatorMaster.configPeakOutputForward(forwardMaxPower);
            elevatorMaster.configPeakOutputReverse(reverseMaxPower);
        }
    }

    int inchesToTicks(double inches) {
        double ticks;

        if (inches <= cascadeOffset) {
            ticks = inches * countsPerInchBeforeCascade;
        } else {
            ticks = ((inches - cascadeOffset) * countsPerInchAfterCascade) + (cascadeOffset * countsPerInchBeforeCascade);
        }

        return (int) ticks;
    }

    public void test(double power) {
        elevatorMaster.set(ControlMode.PercentOutput, power);
    }

    public void hab() {
        reverseMaxPower = -.7;

        elevatorMaster.configPeakOutputReverse(-.7);
    }
}