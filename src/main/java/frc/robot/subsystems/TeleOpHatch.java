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
    public TalonSRX motor;

    static final double countsPerOSRev = 4096.0;
    static final double chainReduction = 22.0/18;
    static final double countsPerDegree = (countsPerOSRev / 360) * chainReduction;

    public TeleOpHatch () {
        motor = new TalonSRX(RobotMap.hatchIntake);

        zero();

        motor.configFactoryDefault();

        motor.setSensorPhase(true);

        motor.config_kP(0, 1.5);
        motor.config_kI(0, 0);
        motor.config_kD(0, 50);
        motor.config_kF(0, 0);

        motor.configClosedloopRamp(.33);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    public void hatchToDegrees(double target) {
        double targetTicks = (target * countsPerDegree);

        motor.set(ControlMode.Position, -targetTicks);
    }

    public void testHatch (double power) {
         motor.set(ControlMode.PercentOutput, power) ;
    }

    public  void zero() {
        motor.setSelectedSensorPosition(0);
    }
}
