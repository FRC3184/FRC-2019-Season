/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class TeleOpDriveTrain extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    public TalonSRX leftMaster;
    public TalonSRX rightMaster;
    private VictorSPX leftSlave;
    private VictorSPX rightSlave;

    private static final double minPower = .1;
    private static final double maxPower = .1;
    private static final double rampRate = .25;

    private static final int ticksPerRev = 4096; //Ticks per output shaft revolution
    private static final double wheelDiameter = 0.152; //Meters
    private static final double wheelbaseWidth = 0.603; //Track is .55 meters, wheelbase is .295 meters

    private static final double deltaTime = 0.02; //Seconds between loops
    private static final double maxVelocity = 2; //Meters/sec/sec
    private static final double maxAcceleration = .75;
    private static final double maxJerk = 60.0456;

    private AHRS m_navX;

    private EncoderFollower leftFollower;
    private EncoderFollower rightFollower;

    private boolean finished = false;

    private double gyroOffset = 0;

    private boolean m_LimelightHasValidTarget = false;
    private double m_LimelightDriveCommand = 0.0;
    private double m_LimelightSteerCommand = 0.0;

    public TeleOpDriveTrain() {
        leftMaster = new TalonSRX(RobotMap.leftDriveMaster);
        rightMaster = new TalonSRX(RobotMap.rightDriveMaster);
        leftSlave = new VictorSPX(RobotMap.leftDriveSlave);
        rightSlave = new VictorSPX(RobotMap.rightDriveSlave);

        leftMaster.configFactoryDefault();
        rightMaster.configFactoryDefault();
        leftSlave.configFactoryDefault();
        rightSlave.configFactoryDefault();

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        leftMaster.configOpenloopRamp(rampRate);
        rightMaster.configOpenloopRamp(rampRate);

        m_navX = new AHRS(RobotMap.gyroPort);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here. <---- Broken?
        //setDefaultCommand(new TeleopDriveTrain());
    }

    public void arcadeDrive(double power, double turn) {
        //robotDrive.arcadeDrive(-power, turn, true);

        double leftPower = power - turn;
        double rightPower = power + turn;

        leftMaster.set(ControlMode.PercentOutput, leftPower);
        rightMaster.set(ControlMode.PercentOutput, rightPower);
    }

    public void tankDrive(double leftPower, double rightPower) {
        leftMaster.set(ControlMode.PercentOutput, leftPower);
        rightMaster.set(ControlMode.PercentOutput, rightPower);
    }

    public void setupPath(double xOffset, double yOffset, double startAngle) {
        leftMaster.setSelectedSensorPosition(0);
        rightMaster.setSelectedSensorPosition(0);
        zeroGyro(startAngle);

        // 3 Waypoints, x (in meters), y (in meters), exit angle (radians)
        Waypoint[] points = new Waypoint[] {
                new Waypoint(0, 0, Pathfinder.d2r(startAngle)),
                new Waypoint(xOffset, yOffset, Pathfinder.d2r(0))
        };

        // Create the Trajectory Configuration4
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
        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_FAST, deltaTime, maxVelocity, maxAcceleration, maxJerk);

        // Generate the trajectory
        Trajectory trajectory = Pathfinder.generate(points, config);

        // Create the Modifier Object
        TankModifier modifier = new TankModifier(trajectory);

        // Generate the Left and Right trajectories using the original trajectory
        // as the centre
        modifier.modify(wheelbaseWidth);

        leftFollower = new EncoderFollower(modifier.getLeftTrajectory());
        rightFollower = new EncoderFollower(modifier.getRightTrajectory());

        leftFollower.configureEncoder(0, ticksPerRev, wheelDiameter);
        // You must tune the PID values on the following line!
        leftFollower.configurePIDVA(.5, 0.0, 0.0, .7508, .1678);

        rightFollower.configureEncoder(0, ticksPerRev, wheelDiameter);
        // You must tune the PID values on the following line!
        rightFollower.configurePIDVA(.5, 0.0, 0.0, .7394, .2182);
    }

    public void followPath() {
        double left_speed = leftFollower.calculate(getLeftEncoderPos());
        double right_speed = rightFollower.calculate(getRightEncoderPos());
        double heading = getSelectedGyroValue();
        double desired_heading = Pathfinder.r2d(leftFollower.getHeading());
        double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);

        /**heading_difference = heading_difference % 360.0;
        if (Math.abs(heading_difference) > 180.0) {
            heading_difference = (heading_difference > 0) ? heading_difference - 360 : heading_difference + 360;
        }*/

        double turn =  .01 * heading_difference;

        SmartDashboard.putNumber("Gyro", heading);
        SmartDashboard.putNumber("Left speed", left_speed);
        SmartDashboard.putNumber("Right speed", right_speed);
        SmartDashboard.putNumber("Left Encoder", leftMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber("Right Encoder", rightMaster.getSelectedSensorPosition());

        leftMaster.set(ControlMode.PercentOutput, (left_speed + turn)); //+ turn -turn
        rightMaster.set(ControlMode.PercentOutput, -(right_speed - turn)); //- turn -turn
    }

    /**Method used by system to check if FOLLOWERS are finished
     *
     * @return Boolean whether followers are finished
     */
    public boolean pathComplete() {
        //AND instead of OR operator?
        return leftFollower.isFinished() || rightFollower.isFinished();
    }

    public void limeLightTrack() {
        SmartDashboard.putBoolean("Entered Method", true);

        // These numbers must be tuned for your Robot!  Be careful!
        final double STEER_K = 0.03;                    // how hard to turn toward the target
        final double DRIVE_K = 0.1;                    // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 5.0;        // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.2;                   // Simple speed limit so we don't drive too fast

        double tv = NetworkTableInstance.getDefault().getTable("limelight-blaze").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight-blaze").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight-blaze").getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight-blaze").getEntry("ta").getDouble(0);

        SmartDashboard.putNumber("TV", tv)
;
        SmartDashboard.putBoolean("Before If", true);

        if (tv < 1.0)
        {
            SmartDashboard.putBoolean("In If", true);
            m_LimelightHasValidTarget = false;
            m_LimelightDriveCommand = 0.0;
            m_LimelightSteerCommand = 0.0;

            SmartDashboard.putBoolean("Before Return", true);
            return;
        }

        SmartDashboard.putBoolean("past Return", true);

        m_LimelightHasValidTarget = true;

        // Start with proportional steering
        double steer_cmd = tx * STEER_K;
        m_LimelightSteerCommand = steer_cmd;

        // try to drive forward until the target area reaches our desired area
        double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE)
        {
            drive_cmd = MAX_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd;

        if (m_LimelightHasValidTarget)
        {
            arcadeDrive(m_LimelightSteerCommand, -m_LimelightDriveCommand);
        }
        else
        {
            arcadeDrive(0.0,0.0);
        }
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

    public void stop() {
        leftMaster.set(ControlMode.PercentOutput, 0);
        rightMaster.set(ControlMode.PercentOutput, 0);
    }

    public int getLeftEncoderPos() {
        return leftMaster.getSelectedSensorPosition();
    }

    public int getRightEncoderPos() {
        return -rightMaster.getSelectedSensorPosition();
    }

    public double getSelectedGyroValue() {
        return m_navX.getYaw() + gyroOffset;
    }

    public boolean gyroCalibrated() {
        return !m_navX.isCalibrating();
    }

    public void zeroGyro(double offset) {
        m_navX.zeroYaw();

        gyroOffset = offset;
    }

    public double clip(double value, double min, double max) {
        if (value <= min) {
            return min;
        } else if (value >= max) {
            return max;
        } else{
            return value;
        }
    }

    public double range(double value, double currentMin, double currentMax, double desiredMin, double desiredMax) {
        return desiredMin + (value - currentMin) * (desiredMax - desiredMin) / (currentMax - currentMin);
    }

    public double sqrInput(double value) {
        if (value > 0) {
            return value * value;
        } else if (value < 0) {
            return -value * value;
        } else {
            return 0;
        }
    }
}