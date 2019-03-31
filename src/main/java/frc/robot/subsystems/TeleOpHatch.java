/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class TeleOpHatch extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public TalonSRX motor;
    public DigitalInput forwardLimitSwitch;
    public DigitalInput reverseLimitSwitch;

    public static final double forwardLimitDegrees = 330;
    public static final double reverseLimitDegrees = 0;

    static final double forwardMaxPower = 1.0; //Hatch out
    static final double reverseMaxPower = -1.0; //Hatch in
    static final double countsPerOSRev = 4096.0;
    static final double chainReduction = 22.0/18;
    static final double countsPerDegree = (countsPerOSRev / 360) * chainReduction;

    public TeleOpHatch () {
        motor = new TalonSRX(RobotMap.hatchIntake);

        forwardLimitSwitch = new DigitalInput(RobotMap.hatchLimitSwitchForward);
        reverseLimitSwitch = new DigitalInput(RobotMap.hatchLimitSwitchReverse);

        motor.configFactoryDefault();

        motor.setSensorPhase(true);

        motor.setSelectedSensorPosition(0);

        motor.config_kP(0, 1.5);
        motor.config_kI(0, 0);
        motor.config_kD(0, 50);
        motor.config_kF(0, 0);

        motor.configClosedloopRamp(.33);

        motor.configPeakOutputForward(forwardMaxPower);
        motor.configPeakOutputReverse(reverseMaxPower);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    public void hatchToDegrees(double target) {
        motor.set(ControlMode.Position, ticksToDegrees(target));
    }

    public int ticksToDegrees(double degrees) {
        return (int) (degrees * countsPerDegree);
    }

    public void testHatch (double power) {
         motor.set(ControlMode.PercentOutput, power) ;
    }

    public void testSwitches() {
        if (!forwardLimitSwitch.get()) {
            motor.setSelectedSensorPosition(ticksToDegrees(forwardLimitDegrees));

            motor.configPeakOutputForward(0);
            motor.configPeakOutputReverse(reverseMaxPower);
        } else if (!reverseLimitSwitch.get()) {
            motor.setSelectedSensorPosition(ticksToDegrees(reverseLimitDegrees));

            motor.configPeakOutputForward(forwardMaxPower);
            motor.configPeakOutputReverse(0);
        } else {
            motor.configPeakOutputForward(forwardMaxPower);
            motor.configPeakOutputReverse(reverseMaxPower);
        }
    }
}
