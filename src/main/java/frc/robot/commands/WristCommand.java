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
import frc.robot.subsystems.TeleOpWrist;

/**
 * An example command.  You can replace me with your own command.
 */
public class WristCommand extends Command {
    private TeleOpWrist wrist;

    public WristCommand(TeleOpWrist wrist) {
        // Use requires() here to declare subsystem dependencies
        requires(wrist);
        this.wrist = wrist;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        if (OI.get().defenciveWristPos()) {
            wrist.wristToPosition(-18);
        } else if (OI.get().wristGround()) {
            wrist.wristToPosition(90);
        } else if (OI.get().wristStowed()) {
            wrist.wristToPosition(-3 );
        } else if (OI.get().wristHatch()) {
            wrist.wristToPosition(60);
        } else if (OI.get().wristCargo()) {
            wrist.wristToPosition(50);
        } else if (OI.get().wristToSwitch()) {
            wrist.runToReverseSwitch();
        }

        wrist.testSwitches();

        SmartDashboard.putBoolean("Wrist Reverse Switch", wrist.reverseSwitch.get());
        SmartDashboard.putBoolean("Wrist Forward Switch", wrist.forwardSwitch.get());
        SmartDashboard.putNumber("Wrist Encoder", wrist.wristMotor.getEncoder().getPosition());
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
