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
import frc.robot.subsystems.TeleOpHab;

/**
 * An example command.  You can replace me with your own command.
 */
public class HabCommand extends Command {
    private TeleOpHab hab;

    private static final double drivePower = .5;
    private static final double turnPower = .2;
    private static final double flipperDeployedDegrees = 100;

    public HabCommand(TeleOpHab hab) {
        // Use requires() here to declare subsystem dependencies
        // requires(Robot_Real.m_subsystem);

        this.hab = hab;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        double power = 0;
        double turn = 0;

        if (OI.get().habRetract()) {
            hab.wristToPosition(0);
        } else if (OI.get().habDeploy()) {
            hab.wristToPosition(flipperDeployedDegrees);
        }

        if (OI.get().habDriveForward()) {
            power = drivePower;
        }

        if (OI.get().habDriveBackwords()) {
            power = -drivePower;
        }

        if (OI.get().habLeft()) {
            turn = turnPower;
        }

        if (OI.get().habRight()) {
            turn = -turnPower;
        }

        hab.habDrive(power, turn);

        hab.testSwitches();

        SmartDashboard.putBoolean("Flipper Limit Reverse", hab.reverseSwitch.get());
        SmartDashboard.putBoolean("Flipper Limit Forward", hab.forwardSwitch.get());
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
