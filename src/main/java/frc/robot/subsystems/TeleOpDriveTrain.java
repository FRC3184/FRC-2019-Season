/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

import java.nio.file.Path;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class TeleOpDriveTrain extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    private WPI_TalonSRX leftMaster;
    private WPI_TalonSRX rightMaster;
    VictorSPX leftSlave;
    VictorSPX rightSlave;

    private static final int ticksPerRev = 4096; //Ticks per output shaft revolution
    private static final double wheelDiameter = 0.1524; //Meters
    private static final double wheelbaseWidth = 0.55; //Track is .55 meters, wheelbase is .295 meters

    private static final double deltaTime = 0.02; //Seconds between loops
    private static final double maxVelocity = 2; //Meters/sec/sec
    private static final double maxAcceleration = .75;
    private static final double maxJerk = 60.0456;

    private AHRS m_navX;

    private EncoderFollower leftFollower;
    private EncoderFollower rightFollower;

    boolean finished = false;

    DifferentialDrive robotDrive;

    double gyroOffset = 0;

    public TeleOpDriveTrain() {
        leftMaster = new WPI_TalonSRX(RobotMap.leftDriveMaster);
        rightMaster = new WPI_TalonSRX(RobotMap.rightDriveMaster);
        leftSlave = new VictorSPX(RobotMap.leftDriveSlave);
        rightSlave = new VictorSPX(RobotMap.rightDriveSlave);

        leftMaster.configFactoryDefault();
        rightMaster.configFactoryDefault();
        leftSlave.configFactoryDefault();
        rightSlave.configFactoryDefault();

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        robotDrive = new DifferentialDrive(leftMaster, rightMaster);

        m_navX = new AHRS(RobotMap.gyroPort);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here. <---- Broken?
        //setDefaultCommand(new TeleopDriveTrain());
    }

    public void arcadeDrive(double power, double turn) {
        robotDrive.arcadeDrive(-power, turn);
    }

    public void tankDrive(double leftPower, double rightPower) {
        robotDrive.tankDrive(-leftPower, -rightPower);
    }

    public void setupPath(double x, double y, double startAngle, double gyroOffset) {
        leftMaster.setSelectedSensorPosition(0);
        rightMaster.setSelectedSensorPosition(0);
        zeroGyro(gyroOffset);

        // 3 Waypoints, x (in meters), y (in meters), exit angle (radians)
        Waypoint[] points = new Waypoint[] {
                new Waypoint(0, 0, Pathfinder.d2r(startAngle)),
                new Waypoint(x, y, Pathfinder.d2r(0))
        };

        // Create the Trajectory Configuration
        //
        // Arguments:
        // Fit Method:          HERMITE_CUBIC or HERMITE_QUINTIC
        // Sample Count:        SAMPLES_HIGH (100 000)
        //                      SAMPLES_LOW  (10 000)
        //                      SAMPLES_FAST (1 000)
        // Time Step:           0.05 Seconds
        // Max Velocity:        1.7 m/s
        // Max Acceleration:    2.0 m/s/s
        // Max Jerk:            60.0 m/s/s/s
        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_FAST, deltaTime, maxVelocity, maxAcceleration, maxJerk);

        // Generate the trajectory
        Trajectory trajectory = Pathfinder.generate(points, config);

        // Create the Modifier Object
        TankModifier modifier = new TankModifier(trajectory);

        // Generate the Left and Right trajectories using the original trajectory
        // as the centre
        modifier.modify(wheelbaseWidth);

        leftFollower = new EncoderFollower(modifier.getLeftTrajectory());
        rightFollower = new EncoderFollower(modifier.getRightTrajectory());

        leftFollower.configureEncoder(getLeftEncoderPos(), ticksPerRev, wheelDiameter);
        // You must tune the PID values on the following line!
        leftFollower.configurePIDVA(.5, 0.0, 0.0, .21063, .067941);

        rightFollower.configureEncoder(getRightEncoderPos(), ticksPerRev, wheelDiameter);
        // You must tune the PID values on the following line!
        rightFollower.configurePIDVA(.5, 0.0, 0.0, .20054, .069116);
    }

    public void followPath() {
        double left_speed = leftFollower.calculate(getLeftEncoderPos());
        double right_speed = rightFollower.calculate(getRightEncoderPos());
        double heading = getSelectedGyroValue();
        double desired_heading = Pathfinder.r2d(leftFollower.getHeading());
        double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
        double turn =  .01 * heading_difference;

        SmartDashboard.putNumber("Gyro", heading);
        SmartDashboard.putNumber("Left speed", left_speed);
        SmartDashboard.putNumber("Right speed", right_speed);
        SmartDashboard.putNumber("Left Encoder", leftMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber("Right Encoder", rightMaster.getSelectedSensorPosition());

        leftMaster.set(ControlMode.PercentOutput, (left_speed) + turn); //+ turn -turn
        rightMaster.set(ControlMode.PercentOutput, -(right_speed) + turn); //- turn -turn
    }

    /**Method used by system to check if FOLLOWERS are finished
     *
     * @return Boolean whether followers are finished
     */
    public boolean pathComplete() {
        //AND instead of OR operator?
        return leftFollower.isFinished() || rightFollower.isFinished();
    }

    public void aimAtTarget(double x) {
        float Kp = -0.01f;
        float min_command = 0.09f;

        double heading_error = -x;
        double steering_adjust = 0.0f;

        if (x > .5f) {
            steering_adjust = Kp*heading_error + min_command;
        } else if (x < -.5f) {
            steering_adjust = Kp*heading_error - min_command;
        } else {
        }

        leftMaster.set(ControlMode.PercentOutput, +steering_adjust);
        rightMaster.set(ControlMode.PercentOutput, steering_adjust);
    }

    public boolean onTarget(double x) {
        boolean aimed = false;

        if (x > .25f) {
            aimed = false;
        } else if (x < -.25f) {
            aimed = false;
        } else {
            aimed = true;
        }

        return aimed;
    }

    public void letGo() {
        leftMaster.set(ControlMode.PercentOutput, 0);
        rightMaster.set(ControlMode.PercentOutput, 0);
    }

    public int getLeftEncoderPos() {
        return -leftMaster.getSelectedSensorPosition();
    }

    public int getRightEncoderPos() {
        return rightMaster.getSelectedSensorPosition();
    }

    public double getSelectedGyroValue() {
        return m_navX.getYaw() - gyroOffset;
    }

    public boolean gyroCalibrated() {
        return !m_navX.isCalibrating();
    }

    public void zeroGyro(double offset) {
        m_navX.zeroYaw();

        gyroOffset = gyroOffset - offset;
    }
}
