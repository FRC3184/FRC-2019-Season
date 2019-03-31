/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.TeleOpDriveTrain;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * An example command.  You can replace me with your own command.
 */
public class TeleopDrive extends Command {
    private TeleOpDriveTrain drive;

    private double KpAim;
    private double KpDistance;
    private double min_aim_command;
    private double z;
    private double x;
    private double xDeg;
    private double[] pos;
    private boolean firstRun = true;

    public TeleopDrive(TeleOpDriveTrain drive) {
        // Use requires() here to declare subsystem dependencies
        requires(drive);
        this.drive = drive;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        KpAim = -0.1f;
        KpDistance = -0.1f;
        min_aim_command = 0.05f;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-blaze");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry tpos = table.getEntry("camtran");

        xDeg = tx.getDouble(0.0);
        pos = tpos.getDoubleArray(new double[] {0,0,0,0,0,0,0,0,0,0,0,0,0,0});

        z = pos[2];
        x = pos[0];

        if (OI.get().limeLight()) {
            if (!drive.onTarget(xDeg) && firstRun) {
                drive.aimAtTarget(xDeg);
            } else if (firstRun) {
                double angleToTarget = Math.atan(z / x);

                if (x > 0) {
                    angleToTarget = +angleToTarget;
                } else if ( x < 0) {
                    angleToTarget = -angleToTarget;
                } else {
                    angleToTarget = 0;
                }

                double startingAngle = 90 - angleToTarget;

                //elevator.setupPath(-(z + 30) * .0254, (x) * .0254, 0 , angleToTarget);

                drive.setupPath(4, 0, 0,0);

                firstRun = false;
            } else if (!drive.pathComplete()){
                drive.followPath();
            }

            /*if (elevator.gyroCalibrated()) {
             if (firstRun) {
             double degree = Math.atan(z/x);
             SmartDashboard.putNumber("Robot offset from target", degree);
             elevator.setupPath(z, x, degree);
             firstRun = false;
             } else if (!elevator.pathComplete()) {
             elevator.followPath();
             } else {
             //Manipulator decision here
             }
             }*/
        } else {
            double power;
            double turn;

            if (OI.get().driveSlowdown()) {
                power = OI.get().scale(OI.get().slowPower(), -1.0, 1.0, -.3, .3);
                turn = OI.get().scale(OI.get().slowTurn(), -1.0, 1.0, -.3, .3);
            } else {
                power = OI.get().getPower();
                turn = OI.get().getTurn();
            }

            if (OI.get().habDriveForward()) {
                turn = .25;
            } else if (OI.get().habDriveBackwords()) {
                turn = -.25;
            }

            drive.arcadeDrive(power, turn);
            firstRun = true;
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
        drive.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}