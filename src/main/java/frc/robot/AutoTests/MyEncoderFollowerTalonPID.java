package frc.robot.AutoTests;

import jaci.pathfinder.Trajectory;

/**
 * The EncoderFollower is an object designed to follow a trajectory based on encoder input. This class can be used
 * for Tank or Swerve drive implementations.
 *
 * @author Jaci
 */
public class MyEncoderFollowerTalonPID {

    int encoder_offset, encoder_tick_count;
    double wheel_circumference;

    //double kp, ki, kd, kv, ka;

    double heading;

    int segment;
    Trajectory trajectory;

    public MyEncoderFollowerTalonPID(Trajectory traj) {
        this.trajectory = traj;
    }

    public MyEncoderFollowerTalonPID() { }

    /**
     * Set a new trajectory to follow, and reset the cumulative errors and segment counts
     * @param traj a previously generated trajectory
     */
    public void setTrajectory(Trajectory traj) {
        this.trajectory = traj;
        reset();
    }

    /**
     * Configure the Encoders being used in the follower.
     * @param initial_position      The initial 'offset' of your encoder. This should be set to the encoder value just
     *                              before you start to track
     * @param ticks_per_revolution  How many ticks per revolution the encoder has
     * @param wheel_diameter        The diameter of your wheels (or pulleys for track systems) in meters
     */
    public void configureEncoder(int initial_position, int ticks_per_revolution, double wheel_diameter) {
        encoder_offset = initial_position;
        encoder_tick_count = ticks_per_revolution;
        wheel_circumference = Math.PI * wheel_diameter;
    }

    /**
     * Reset the follower to start again. Encoders must be reconfigured.
     */
    public void reset() {
        segment = 0;
    }

    /**
     * Calculate the desired output for the motors, based on the amount of ticks the encoder has gone through.
     * This does not account for heading of the robot. To account for heading, add some extra terms in your control
     * loop for realignment based on gyroscope input and the desired heading given by this object.
     * @param encoder_tick The amount of ticks the encoder has currently measured.
     * @return             The desired output for your motor controller
     */
    public double calculate(int encoder_tick) {
        // Number of Revolutions * Wheel Circumference
        double distance_covered = ((double)(encoder_tick - encoder_offset) / encoder_tick_count)
                * wheel_circumference;
        if (segment < trajectory.length()) {
            Trajectory.Segment seg = trajectory.get(segment);
            double error = seg.position - distance_covered;
            heading = seg.heading;
            segment++;

            return error;
        } else return 0;
    }

    /**
     * @return the desired heading of the current point in the trajectory
     */
    public double getHeading() {
        return heading;
    }

    /**
     * @return the current segment being operated on
     */
    public Trajectory.Segment getSegment() {
        return trajectory.get(segment);
    }

    /**
     * @return whether we have finished tracking this trajectory or not.
     */
    public boolean isFinished() {
        return segment >= trajectory.length();
    }

}
