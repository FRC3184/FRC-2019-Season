/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AutoTests.*;
import frc.robot.commands.CargoHolder;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.HatchHolder;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.TeleOpCargo;
import frc.robot.subsystems.TeleOpDriveTrain;
import frc.robot.subsystems.TeleOpElevator;
import frc.robot.subsystems.TeleOpHatch;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private TeleOpDriveTrain teleOpDriveTrain;
    private TeleopDrive teleopDrive;
    private ElevatorCommand elevatorCommand;
    private TeleOpElevator teleOpElevator;
    private HatchHolder hatchCommand;
    private TeleOpHatch teleOpHatch;
    private CargoHolder cargoCommand;
    private TeleOpCargo teleOpCargo;

    private Command selectedAutoCommand;

    SendableChooser<Command> m_chooser = new SendableChooser<>();

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        Scheduler.getInstance().enable();

        teleOpDriveTrain = new TeleOpDriveTrain();
        //teleOpElevator = new TeleOpElevator();
        teleOpHatch = new TeleOpHatch();
        teleOpCargo = new TeleOpCargo();

        teleopDrive = new TeleopDrive(teleOpDriveTrain);
        //elevatorCommand = new ElevatorCommand(teleOpElevator);
        hatchCommand = new HatchHolder(teleOpHatch);
        cargoCommand = new CargoHolder(teleOpCargo);

        m_chooser.setDefaultOption("Default Test", new DefaultTestAuto());
        m_chooser.addOption("Default Test Waypoints", new DefaultTestAutoWaypoints());

        SmartDashboard.putData("Auto mode select", m_chooser);
    }

    @Override
    public void testInit() {

    }

    @Override
    public void teleopInit() {
        teleopDrive.start();
        //elevatorCommand.start();
        //hatchCommand.start();
        //cargoCommand.start();
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString code to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional commands to the
     * chooser code above (like the commented example) or additional comparisons
     * to the switch structure below with additional strings & commands.
     */
    @Override
    public void autonomousInit() {
        selectedAutoCommand = m_chooser.getSelected();

        selectedAutoCommand.start();

        //teleopDrive.start();
        //elevatorCommand.start();
        //hatchCommand.start();
        //cargoCommand.start();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     * You can use it to reset any subsystem information you want to clear when
     * the robot is disabled.
     */
    @Override
    public void disabledInit() { }

    /**
     * This function is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() { }

    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() { Scheduler.getInstance().run(); }
}