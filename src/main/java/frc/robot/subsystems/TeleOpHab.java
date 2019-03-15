/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class TeleOpHab extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    private CANSparkMax flipper;
    private CANSparkMax leftWheel;
    private CANSparkMax rightWheel;

    private CANPIDController flipperPID;
    private CANEncoder flipperEncoder;

    public DigitalInput forwardSwitch;
    public DigitalInput reverseSwitch;

    private static final double flipperMaxPower = .1;
    private static final double maxForwardDegrees = 100;
    private static final double maxReverseDegrees = 0;

    private static final double driveRamp = .25;

    private static final int ticksPerFlipperDegree = (int) (1.0 * 250.0) / 360;

    public TeleOpHab() {
        flipper = new CANSparkMax(RobotMap.habFlipper, CANSparkMaxLowLevel.MotorType.kBrushless);
        leftWheel = new CANSparkMax(RobotMap.habLeftWheel, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightWheel = new CANSparkMax(RobotMap.habRightWheel, CANSparkMaxLowLevel.MotorType.kBrushless);

        forwardSwitch = new DigitalInput(RobotMap.habFlipperLimitSwitchForward);
        reverseSwitch = new DigitalInput(RobotMap.habFlipperLimitSwitchReverse);

        flipper.restoreFactoryDefaults();
        leftWheel.restoreFactoryDefaults();
        rightWheel.restoreFactoryDefaults();

        flipperPID = flipper.getPIDController();
        flipperEncoder = flipper.getEncoder();

        zero();

        flipperPID.setP(.070);
        flipperPID.setI(0);
        flipperPID.setD(2);
        flipperPID.setFF(0);
        flipperPID.setIZone(0);
        flipperPID.setOutputRange(-flipperMaxPower, flipperMaxPower);
        flipper.setClosedLoopRampRate(.33);

        leftWheel.setOpenLoopRampRate(driveRamp);
        rightWheel.setOpenLoopRampRate(driveRamp);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    public void wristToPosition(double targetDegrees) {
        flipperPID.setReference(-targetDegrees * ticksPerFlipperDegree, ControlType.kPosition);
    }

    public void testSwitches() {
        if (!forwardSwitch.get()) {
            flipper.getEncoder().setPosition(degreesToTicks(maxForwardDegrees));
            flipperPID.setOutputRange(-flipperMaxPower, 0);
        } else if (!reverseSwitch.get()) {
            flipper.getEncoder().setPosition(degreesToTicks(maxReverseDegrees));
            flipperPID.setOutputRange(0, flipperMaxPower);
        } else {
            flipperPID.setOutputRange(-flipperMaxPower, flipperMaxPower);
        }
    }

    double degreesToTicks(double degrees) {
        return -degrees * ticksPerFlipperDegree;
    }

    public void habDrive(double power, double turn) {
        double leftPower = power - turn;
        double rightPower = power + turn;

        leftWheel.set(leftPower);
        rightWheel.set(-rightPower);
    }

    public void test(double power) {
        flipper.set(power);
    }

    public void zero() {
        flipper.getEncoder().setPosition(0);
    }
}
