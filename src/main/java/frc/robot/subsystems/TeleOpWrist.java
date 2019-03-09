/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANPIDController;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class TeleOpWrist extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    CANSparkMax wristMotor;
    CANPIDController wristPID;

    int countsPerMotorRev = 1;
    double gearRatio = 213.33;
    double countsPerOutputRev = countsPerMotorRev * gearRatio;
    double countsPerDegree = countsPerOutputRev / 360;

    public TeleOpWrist() {
        wristMotor = new CANSparkMax(RobotMap.wrist, CANSparkMaxLowLevel.MotorType.kBrushless);

        wristMotor.restoreFactoryDefaults();

        wristPID = wristMotor.getPIDController();

        wristMotor.getEncoder().setPosition(0);

        wristPID.setP(.070);
        wristPID.setI(0);
        wristPID.setD(2);
        wristPID.setFF(0);
        wristPID.setIZone(0);
        wristPID.setOutputRange(-1.0, 1.0);
        wristMotor.setClosedLoopRampRate(.33);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new TeleopDrive());
    }

    public void wristToPosition(double targetDegrees) {
        wristPID.setReference(-targetDegrees * countsPerDegree, ControlType.kPosition);
    }

    public void testSwitches() {
        if (elevatorLimitSwitches.isFwdLimitSwitchClosed()) {
            elevatorMaster.setSelectedSensorPosition(0);
        } else if (elevatorLimitSwitches.isRevLimitSwitchClosed()) {
            wristMotor.getEncoder().setPosition(inchesToDegrees(topInches));
        }
    }

    public void test(double power) {
        wristMotor.set(power);
    }

    public void zero() {
        wristMotor.getEncoder().setPosition(0);
    }
}
