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

    private static final double drivePower = 1.0;
    private static final double turnPower = .8;
    private static final double hab2 = 7;
    private static final double hab3 = 21;
    private double habPosition = hab3;
    private boolean togglePressed = false;
    private boolean hab3Selected = true;

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
        } else if (OI.get().habDeploy()){
            hab.wristToPosition(habPosition);
        }

        if (OI.get().habToggle() && togglePressed == false) {
            togglePressed = true;
            hab3Selected = !hab3Selected;
        } else if(!OI.get().habToggle() && togglePressed == true){
            togglePressed = false;
        }

        if (hab3Selected) {
            habPosition = hab3;
        } else {
            habPosition = hab2;
        }

        if (OI.get().habDriveForward()) {
            power = drivePower;
        } else if (OI.get().habDriveBackwords()) {
            power = -drivePower;
        } else {
            power = 0;
        }

        if (OI.get().habLeft()) {
            turn = turnPower;
        } else if (OI.get().habRight()) {
            turn = -turnPower;
        } else {
            turn = 0;
        }

        hab.habDrive(-turn, power);

        //hab.test(OI.get().testHab());

        hab.testSwitches();

        hab.updateToPos();

        SmartDashboard.putBoolean("Left Limit Reverse", hab.leftReverseSwitch.get());
        SmartDashboard.putBoolean("Left Limit Forward", hab.leftForwardSwitch.get());
        SmartDashboard.putBoolean("right Limit Reverse", hab.rightReverseSwitch.get());
        SmartDashboard.putBoolean("right Limit Forward", hab.rightForwardSwitch.get());
        SmartDashboard.putNumber("left Stilt", hab.leftStiltEncoder.getPosition());
        SmartDashboard.putNumber("right Stilt", hab.rightStiltEncoder.getPosition());
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
