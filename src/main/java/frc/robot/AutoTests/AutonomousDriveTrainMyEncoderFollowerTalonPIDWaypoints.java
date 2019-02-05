package frc.robot.AutoTests;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

import java.io.File;

public class AutonomousDriveTrainMyEncoderFollowerTalonPIDWaypoints extends Subsystem {

    private static final int k_ticks_per_rev = 4096;
    private static final double k_wheel_diameter = 0.1524;
    private static final double k_max_velocity = 3.048;
    double wheelbase_width = 0.55;

    private TalonSRX m_left_master;
    private TalonSRX m_right_master;
    private VictorSPX m_left_slave;
    private VictorSPX m_right_slave;

    private AHRS m_navX;

    private MyEncoderFollowerTalonPID m_left_follower;
    private MyEncoderFollowerTalonPID m_right_follower;

    boolean m_finished = false;

    public AutonomousDriveTrainMyEncoderFollowerTalonPIDWaypoints() {
        m_left_master = new TalonSRX(RobotMap.leftDriveMaster);
        m_right_master = new TalonSRX(RobotMap.rightDriveMaster);
        m_left_slave = new VictorSPX(RobotMap.leftDriveSlave);
        m_right_slave = new VictorSPX(RobotMap.rightDriveSlave);
        m_navX = new AHRS(SPI.Port.kMXP);

        m_left_slave.follow(m_left_master);
        m_right_slave.follow(m_right_master);
    }

    @Override
    public void initDefaultCommand() {
        //setDefaultCommand(new AutoDriveDefault(this));
    }

    public void setupPath(String pathName) {
        // 3 Waypoints
        Waypoint[] points = new Waypoint[] {
                new Waypoint(6.5, 0, Pathfinder.d2r(0)),      // Waypoint @ x=-4, y=-1, exit angle=-45 degrees
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
        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);

        // Generate the trajectory
        Trajectory trajectory = Pathfinder.generate(points, config);

        // Create the Modifier Object
        TankModifier modifier = new TankModifier(trajectory);

        // Generate the Left and Right trajectories using the original trajectory
        // as the centre
        modifier.modify(wheelbase_width);

        m_left_follower = new MyEncoderFollowerTalonPID(modifier.getLeftTrajectory());
        m_right_follower = new MyEncoderFollowerTalonPID(modifier.getRightTrajectory());

        m_left_follower.configureEncoder(getLeftEncoderPos(), k_ticks_per_rev, k_wheel_diameter);
        // You must tune the PID values on the following line!
        m_left_master.config_kF(0, 0.1097);
        m_left_master.config_kP(0, 1.0);
        m_left_master.config_kI(0, 0.0);
        m_left_master.config_kD(0, 0.0);
        //m_left_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / k_max_velocity, 0);

        m_right_follower.configureEncoder(getRightEncoderPos(), k_ticks_per_rev, k_wheel_diameter);
        // You must tune the PID values on the following line!
        m_left_master.config_kF(0, 0.1097);
        m_left_master.config_kP(0, 1.0);
        m_left_master.config_kI(0, 0.0);
        m_left_master.config_kD(0, 0.0);
        //m_right_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / k_max_velocity, 0);
    }

    public void followPath() {
        double left_count = m_left_follower.calculate(getLeftEncoderPos());
        double right_count = m_right_follower.calculate(getRightEncoderPos());
        double heading = getSelectedGyroValue();
        double desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
        double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
        double turn =  0.8 * (-1.0/80.0) * heading_difference;

        m_left_master.set(ControlMode.MotionMagic, left_count + turn);
        m_right_master.set(ControlMode.MotionMagic, right_count - turn);
    }

    /**Method used by system to check if FOLLOWERS are finished
     *
     * @return
     */
    public boolean pathCompete() {
        //AND instead of OR operator?
        return m_left_follower.isFinished() || m_right_follower.isFinished();
    }

    public void letGo() {
        m_left_master.set(ControlMode.PercentOutput, 0);
        m_right_master.set(ControlMode.PercentOutput, 0);
    }

    public int getLeftEncoderPos() {
        return m_left_master.getSelectedSensorPosition();
    }

    public int getRightEncoderPos() {
        return m_right_master.getSelectedSensorPosition();
    }

    public float getSelectedGyroValue() {
        return m_navX.getYaw();
    }
}
