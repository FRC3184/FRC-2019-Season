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
    TeleOpElevator drive;

    public ElevatorCommand(TeleOpElevator drive) {
        requires(drive);
        this.drive = drive;

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
        if (OI.get().elevatorCargoMid()) {
            drive.elevatorMoveToInches(20);
        } else if (OI.get().elevatorHatchLow() >= .5) {
            drive.elevatorMoveToInches(0);
        } else if (OI.get().elevatorCargoHigh()) {
            drive.elevatorMoveToInches(40);
        } else if (OI.get().elevatorCargoLow()) {
            drive.elevatorMoveToInches(5);
        } else if (OI.get().elevatorCargoMid()) {
            drive.elevatorMoveToInches(27);
        } else if (OI.get().elevatorHatchMid()) {
            drive.elevatorMoveToInches(22);
        } else if (OI.get().elevatorHatchHigh()) {
            drive.elevatorMoveToInches(35);
        } else if (OI.get().elevatorCargoHP()) {
            drive.elevatorMoveToInches(15);
        } else if (OI.get().elevatorCargoShip()) {
            drive.elevatorMoveToInches(10);
        }
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
