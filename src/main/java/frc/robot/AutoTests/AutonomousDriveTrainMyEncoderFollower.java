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

import java.io.File;

public class AutonomousDriveTrainMyEncoderFollower extends Subsystem {

    private static final int k_ticks_per_rev = 4096;
    private static final double k_wheel_diameter = 6.0 / 12.0;
    private static final double k_max_velocity = 10;

    private TalonSRX m_left_master;
    private TalonSRX m_right_master;
    private VictorSPX m_left_slave;
    private VictorSPX m_right_slave;

    private AHRS m_navX;

    private MyEncoderFollower m_left_follower;
    private MyEncoderFollower m_right_follower;

    boolean m_finished = false;

    public AutonomousDriveTrainMyEncoderFollower() {
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
        Trajectory left_trajectory = Pathfinder.readFromCSV(new File("/home/lvuser/deploy/output/" + pathName + ".left" + ".pf1.csv"));
        Trajectory right_trajectory = Pathfinder.readFromCSV(new File("/home/lvuser/deploy/output/" + pathName + ".right" + ".pf1.csv"));

        m_left_follower = new MyEncoderFollower(left_trajectory);
        m_right_follower = new MyEncoderFollower(right_trajectory);

        m_left_follower.configureEncoder(getLeftEncoderPos(), k_ticks_per_rev, k_wheel_diameter);
        // You must tune the PID values on the following line!
        m_left_follower.configurePIDVA(0.5, 0.0, 0.0, 1 / k_max_velocity, 0);

        m_right_follower.configureEncoder(getRightEncoderPos(), k_ticks_per_rev, k_wheel_diameter);
        // You must tune the PID values on the following line!
        m_right_follower.configurePIDVA(0.5, 0.0, 0.0, 1 / k_max_velocity, 0);
    }

    public void followPath() {
        double left_speed = m_left_follower.calculate(getLeftEncoderPos());
        double right_speed = m_right_follower.calculate(getRightEncoderPos());
        double heading = getSelectedGyroValue();
        double desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
        double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
        double turn =  0.8 * (-1.0/80.0) * heading_difference;

        m_left_master.set(ControlMode.PercentOutput, left_speed + turn);
        m_right_master.set(ControlMode.PercentOutput, -(right_speed) - turn);
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
