/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class TeleOpWrist extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public CANSparkMax wristMotor;
    CANPIDController wristPID;
    public DigitalInput reverseSwitch;
    public DigitalInput forwardSwitch;

    int countsPerMotorRev = 1;
    double gearRatio = 213.33;
    double countsPerOutputRev = countsPerMotorRev * gearRatio;
    double countsPerDegree = countsPerOutputRev / 360;

    static final double maxPower = .1;
    double maxDegrees = 90;
    double rearMax = -0;

    public TeleOpWrist() {
        wristMotor = new CANSparkMax(RobotMap.wrist, CANSparkMaxLowLevel.MotorType.kBrushless);

        reverseSwitch = new DigitalInput(RobotMap.wristLimitSwitchReverse);
        forwardSwitch = new DigitalInput(RobotMap.wristLimitSwitchForward);

        wristMotor.restoreFactoryDefaults();

        wristPID = wristMotor.getPIDController();

        wristMotor.getEncoder().setPosition(degreesToTicks(rearMax));

        wristPID.setP(.070);
        wristPID.setI(0);
        wristPID.setD(2);
        wristPID.setFF(0);
        wristPID.setIZone(0);
        wristPID.setOutputRange(-maxPower, maxPower);
        wristMotor.setClosedLoopRampRate(.33);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new DriveTest());
    }

    public void wristToPosition(double targetDegrees) {
        wristPID.setReference(-targetDegrees * countsPerDegree, ControlType.kPosition);
    }

    public void testSwitches() {
        if (!forwardSwitch.get()) {
            wristMotor.getEncoder().setPosition(degreesToTicks(rearMax));
            wristPID.setOutputRange(-maxPower, 0);
        } else if (!reverseSwitch.get()) {
            wristMotor.getEncoder().setPosition(degreesToTicks(maxDegrees));
            wristPID.setOutputRange(0, maxPower);
        } else {
            wristPID.setOutputRange(-maxPower, maxPower);
        }
    }

    double degreesToTicks(double degrees) {
        return -degrees * countsPerDegree;
    }

    public void test(double power) {
        wristMotor.set(power);
    }

    public void zero() {
        wristMotor.getEncoder().setPosition(0);
    }
}
