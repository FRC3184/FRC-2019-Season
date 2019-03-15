/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.subsystems.TeleOpElevator;

/**
 * An example command.  You can replace me with your own command.
 */
public class ElevatorCommand extends Command {
    private TeleOpElevator elevator;

    public ElevatorCommand(TeleOpElevator elevator) {
        requires(elevator);
        this.elevator = elevator;

        // Use requires() here to declare subsystem dependencies
        // requires(Robot_Real.m_subsystem);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        if (OI.get().elevatorCargoHP()) {
            elevator.elevatorMoveToInches(12);
        } else if (OI.get().elevatorCargoShip()) {
            elevator.elevatorMoveToInches(38);
        } else if (OI.get().elevatorHatchLow()) {
            elevator.elevatorMoveToInches(0);
        } else if (OI.get().elevatorHatchMid()) {
            elevator.elevatorMoveToInches(29.5);
        } else if (OI.get().elevatorHatchHigh()) {
            elevator.elevatorMoveToInches(57.5);
        } else if (OI.get().elevatorCargoLow()) {
            elevator.elevatorMoveToInches(13.5);
        } else if (OI.get().elevatorCargoMid()) {
            elevator.elevatorMoveToInches(41.5);
        } else if (OI.get().elevatorCargoHigh()) {
            elevator.elevatorMoveToInches(69.5);
        }

        elevator.testSwitches();

        SmartDashboard.putBoolean("Elevator Switch reverse", elevator.reverseLimitSwitch.get());
        SmartDashboard.putBoolean("Elevator Switch forward", elevator.forwardLimitSwitch.get());
        SmartDashboard.putNumber("Elevator Encoder", elevator.elevatorMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber("elevator target", elevator.targetT);

        /**SmartDashboard.putNumber("Elevator Master Voltage", elevator.elevatorMaster.getMotorOutputVoltage());
        SmartDashboard.putNumber("Elevator Slave Voltage", elevator.elevatorSlave.getMotorOutputVoltage());
        SmartDashboard.putNumber("Elevator Master Amperage", elevator.elevatorMaster.getOutputCurrent());
        SmartDashboard.putNumber("Elevator Slave Amperage", elevator.elevatorSlave.getOutputCurrent());*/
    }
    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
