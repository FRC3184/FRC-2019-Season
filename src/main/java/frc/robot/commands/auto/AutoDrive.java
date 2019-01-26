/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.EncoderPathFollower;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;
import edu.wpi.first.wpilibj.SPI;

/**
 * An example command.  You can replace me with your own command.
 */
public class AutoDrive extends Command {
    Trajectory.Config config;
    Waypoint[] points;
    TankModifier modifier;
    AHRS nav_x;
    EncoderPathFollower left;
    EncoderPathFollower right;

    TalonSRX leftMaster;
    TalonSRX rightMaster;
    VictorSPX leftSlave;
    VictorSPX rightSlave;

    public AutoDrive() {
        leftMaster = new TalonSRX(RobotMap.leftDriveMaster);
        rightMaster = new TalonSRX(RobotMap.rightDriveMaster);
        leftSlave = new VictorSPX(RobotMap.leftDriveSlave);
        rightSlave = new VictorSPX(RobotMap.rightDriveSlave);

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        nav_x = new AHRS(SPI.Port.kMXP);

        // Use requires() here to declare subsystem dependencies
        // requires(Robot.m_subsystem);

        config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.0, 2.0, 60.0);
        points = new Waypoint[] {
                new Waypoint(-4, -1, Pathfinder.d2r(-45)),
                new Waypoint(-2, -2, 0),
                new Waypoint(0, 0, 0)
        };

        DriverStation.reportWarning("Generating Trajectory", false);

        Trajectory trajectory = Pathfinder.generate(points, config);

        // Wheelbase Width = 0.5m
        TankModifier modifier = new TankModifier(trajectory).modify(0.5);

        left = new EncoderPathFollower(modifier.getLeftTrajectory());
        right = new EncoderPathFollower(modifier.getRightTrajectory());
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        // Encoder Position is the current, cumulative position of your encoder. If you're using an SRX, this will be the
        // 'getEncPosition' function.
        // 1000 is the amount of encoder ticks per full revolution
        // Wheel Diameter is the diameter of your wheels (or pulley for a track system) in meters
        left.configureEncoder(leftMaster.getSelectedSensorPosition(), 1000, 6);
        right.configureEncoder(rightMaster.getSelectedSensorPosition(), 1000, 6);

        // The first argument is the proportional gain. Usually this will be quite high
        // The second argument is the integral gain. This is unused for motion profiling
        // The third argument is the derivative gain. Tweak this if you are unhappy with the tracking of the trajectory
        // The fourth argument is the velocity ratio. This is 1 over the maximum velocity you provided in the
        //      trajectory configuration (it translates m/s to a -1 to 1 scale that your motors can read)
        // The fifth argument is your acceleration gain. Tweak this if you want to get to a higher or lower speed quicker
        left.configurePIDVA(1.0, 0.0, 0.0, 1 / 16.5, 0);

        double l = left.calculate(leftMaster.getSelectedSensorPosition());
        double r = right.calculate(rightMaster.getSelectedSensorPosition());

        double gyro_heading = nav_x.getYaw();    // Assuming the gyro is giving a value in degrees
        double desired_heading = Pathfinder.r2d(left.getHeading());  // Should also be in degrees

        double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
        double turn = 0.8 * (-1.0/80.0) * angleDifference;

        leftMaster.set(ControlMode.PercentOutput, l + turn);
        rightMaster.set(ControlMode.PercentOutput, r - turn);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return left.isFinished() && right.isFinished();
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
