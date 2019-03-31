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
import frc.robot.subsystems.TeleOpHatch;

/**
 * An example command.  You can replace me with your own command.
 */
public class HatchCommand extends Command {
    private TeleOpHatch hatch;

    private boolean first = true;

    public HatchCommand(TeleOpHatch hatch) {
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
        if (OI.get().hatchGrab()) {
            hatch.hatchToDegrees(315);

            first = true;
        } else if (OI.get().placeHatch()) {
            if (first) {
                hatch.hatchToDegrees(90);

                if (OI.get().placeHatch()) {
                    first = false;
                }
            } else {
                hatch.hatchToDegrees(0);
            }
        }

        SmartDashboard.putBoolean("Hatch Forward Limit", hatch.forwardLimitSwitch.get());
        SmartDashboard.putBoolean("Hatch Reverse Limit", hatch.reverseLimitSwitch.get());
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
