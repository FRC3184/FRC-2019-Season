/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class TeleOpHab extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    private CANSparkMax leftStilt;
    private CANSparkMax rightStilt;
    private VictorSP leftWheel;
    private VictorSP rightWheel;

    private CANPIDController leftStiltPID;
    public CANEncoder leftStiltEncoder;
    private CANPIDController rightStiltPID;
    public CANEncoder rightStiltEncoder;

    public DigitalInput leftForwardSwitch;
    public DigitalInput leftReverseSwitch;

    public DigitalInput rightForwardSwitch;
    public DigitalInput rightReverseSwitch;

    private static final double stiltMaxPower = 1.0;
    private static final double maxStiltExtend = 23;
    private static final double stiltZero = 0;

    private static final int ticksPerStiltDegree = (int) ((1.0 * 15.0) / 3.71); //Actually inches...

    private static final double stiltP = .5;
    private static final double stiltI = 0;
    private static final double stiltD = 0;
    private static final double stiltFF = 0;

    double runPos = 0;

    public TeleOpHab() {
        leftStilt = new CANSparkMax(RobotMap.leftStilt, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightStilt = new CANSparkMax(RobotMap.rightStilt, CANSparkMaxLowLevel.MotorType.kBrushless);

        leftForwardSwitch = new DigitalInput(RobotMap.habLeftStiltForward);
        leftReverseSwitch = new DigitalInput(RobotMap.habLeftStiltReverse);
        rightForwardSwitch = new DigitalInput(RobotMap.habRightStiltForward);
        rightReverseSwitch = new DigitalInput(RobotMap.habRightStiltReverse);

        leftWheel = new VictorSP(RobotMap.leftWheel);
        rightWheel = new VictorSP(RobotMap.rightWheel);

        leftStilt.restoreFactoryDefaults();
        rightStilt.restoreFactoryDefaults();

        leftStiltPID = leftStilt.getPIDController();
        leftStiltEncoder = leftStilt.getEncoder();
        rightStiltPID = rightStilt.getPIDController();
        rightStiltEncoder = rightStilt.getEncoder();

        zero();

        leftStiltPID.setP(stiltP);
        leftStiltPID.setI(stiltI);
        leftStiltPID.setD(stiltD);
        leftStiltPID.setFF(stiltFF);
        leftStiltPID.setOutputRange(-stiltMaxPower, stiltMaxPower);
        leftStilt.setClosedLoopRampRate(.33);

        rightStiltPID.setP(stiltP);
        rightStiltPID.setI(stiltI);
        rightStiltPID.setD(stiltD);
        rightStiltPID.setFF(stiltFF);
        rightStiltPID.setOutputRange(-stiltMaxPower, stiltMaxPower);
        rightStilt.setClosedLoopRampRate(.33);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    public void wristToPosition(double targetDegrees) {
        runPos = degreesToTicks(targetDegrees);

        //leftStiltPID.setReference(degreesToTicks(targetDegrees), ControlType.kPosition);
        //rightStiltPID.setReference(-degreesToTicks(targetDegrees), ControlType.kPosition);
    }

    public void testSwitches() {
        if (!leftForwardSwitch.get()) {
            leftStilt.getEncoder().setPosition(degreesToTicks(maxStiltExtend));
            leftStiltPID.setOutputRange(-stiltMaxPower, 0);
        } else if (!rightReverseSwitch.get()) {
            leftStilt.getEncoder().setPosition(degreesToTicks(stiltZero));
            leftStiltPID.setOutputRange(0, stiltMaxPower);
        } else {
            leftStiltPID.setOutputRange(-stiltMaxPower, stiltMaxPower);
        }

        if (!leftForwardSwitch.get()) {
            rightStilt.getEncoder().setPosition(degreesToTicks(maxStiltExtend));
            rightStiltPID.setOutputRange(-stiltMaxPower, 0);
        } else if (!leftReverseSwitch.get()) {
            rightStilt.getEncoder().setPosition(degreesToTicks(stiltZero));
            rightStiltPID.setOutputRange(0, stiltMaxPower);
        } else {
            rightStiltPID.setOutputRange(-stiltMaxPower, stiltMaxPower);
        }
    }

    double degreesToTicks(double degrees) {
        return degrees * ticksPerStiltDegree;
    }

    public void habDrive(double power, double turn) {
        double leftPower = power - turn;
        double rightPower = power + turn;

        leftWheel.set(leftPower);
        rightWheel.set(rightPower);
    }

    public void test(double power) {
        leftStilt.set(power);
        rightStilt.set(-power);
    }

    public void zero() {
        leftStilt.getEncoder().setPosition(0);
        rightStilt.getEncoder().setPosition(0);
    }

    public void updateToPos() {
        //down
        if (runPos >= 1) {
            if (leftStilt.getEncoder().getPosition() <= runPos) {
                leftStilt.set(stiltMaxPower);
            } else {
                leftStilt.set(.16);
            }
            //negative
            if (rightStilt.getEncoder().getPosition() >= -runPos) {
                rightStilt.set(-stiltMaxPower);
            } else {
                rightStilt.set(-.16);
            }
        }
        //up
        else if (runPos < 1) {
            if (leftStilt.getEncoder().getPosition() >= runPos) {
                leftStilt.set(-.25);
            } else {
                leftStilt.set(0);
            }

            if (rightStilt.getEncoder().getPosition() <= -runPos) {
                rightStilt.set(.25);
            } else {
                rightStilt.set(0);
            }
        }
    }
}
