/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.subsystems.TeleOpHatch;

/**
 * An example command.  You can replace me with your own command.
 */
public class HatchHolder extends Command {
    TeleOpHatch hatch;

    public HatchHolder(TeleOpHatch hatch) {
        // Use requires() here to declare subsystem dependencies
        // requires(Robot_Real.m_subsystem);
        this.hatch = hatch;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        /**if (OI.get().hatchIntake()) {
            hatch.grabHatch(512);
        } else {
            hatch.placeHatch(0);
        }*/

        hatch.test(OI.get().hatchTest());
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
    }
}
