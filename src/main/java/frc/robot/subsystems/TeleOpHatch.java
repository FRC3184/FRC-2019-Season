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

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class TeleOpHatch extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    TalonSRX motor;

    static final double countsPerOSRev = 4096;
    static final double chainReduction = 18.0/22; //IN INCHES
    static final double countsPerDegree = (countsPerOSRev * chainReduction) / 360;

    public TeleOpHatch () {
        motor = new TalonSRX(RobotMap.hatchIntake);

        motor.configFactoryDefault();

        motor.getSelectedSensorPosition(0);

        motor.config_kP(0, .00001);
        motor.config_kI(0, 0);
        motor.config_kD(0, 0);
        motor.config_kF(0, 0);

        motor.configClosedloopRamp(.33);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    public void hatchToDegrees(double target) {
        double targetTicks = (target* countsPerDegree);

        motor.set(ControlMode.Position, targetTicks);
    }

    public void test (double power) {
        motor.set(ControlMode.PercentOutput, power);
    }
}
