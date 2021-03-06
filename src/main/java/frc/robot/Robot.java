/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.*;
import frc.robot.commands.test.*;
import frc.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private TeleOpDriveTrain teleOpDriveTrain;
    private DriveTest driveTest;
    private DriveCommand driveCommand;
    private ElevatorCommand elevatorCommand;
    private ElevatorTest elevatorTest;
    private TeleOpElevator teleOpElevator;
    private HatchCommand hatchCommand;
    private HatchTest hatchTest;
    private TeleOpHatch teleOpHatch;
    private CargoCommand cargoCommand;
    private CargoTest cargoTest;
    private TeleOpCargo teleOpCargo;
    private WristCommand wristCommand;
    private WristTest wristTest;
    private TeleOpWrist teleOpWrist;
    private HabCommand habCommand;
    private TeleOpHab teleOpHab;

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
        teleOpElevator = new TeleOpElevator();
        teleOpHatch = new TeleOpHatch();
        teleOpCargo = new TeleOpCargo();
        teleOpWrist = new TeleOpWrist();
        teleOpHab = new TeleOpHab();

        driveCommand = new DriveCommand(teleOpDriveTrain);
        driveTest = new DriveTest(teleOpDriveTrain);
        elevatorCommand = new ElevatorCommand(teleOpElevator);
        elevatorTest = new ElevatorTest(teleOpElevator);
        hatchCommand = new HatchCommand(teleOpHatch);
        hatchTest = new HatchTest(teleOpHatch);
        cargoCommand = new CargoCommand(teleOpCargo);
        cargoTest = new CargoTest(teleOpCargo);
        wristCommand = new WristCommand(teleOpWrist);
        wristTest = new WristTest(teleOpWrist);
        habCommand = new HabCommand(teleOpHab);

        /**m_chooser.setDefaultOption("Default Test", new DefaultTestAuto());
        m_chooser.addOption("Default Test Waypoints", new DefaultTestAutoWaypoints());

        SmartDashboard.putData("Auto mode select", m_chooser);*/
    }

    @Override
    public void testInit() {
        driveTest.start();
        elevatorTest.start();
        hatchTest.start();
        cargoTest.start();
        wristTest.start();
    }

    @Override
    public void teleopInit() {
        driveCommand.start();
        elevatorCommand.start();
        hatchCommand.start();
        cargoCommand.start();
        wristCommand.start();
        habCommand.start();
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
        //selectedAutoCommand = m_chooser.getSelected();

        //selectedAutoCommand.start();

        driveCommand.start();
        elevatorCommand.start();
        hatchCommand.start();
        cargoCommand.start();
        wristCommand.start();
        //habCommand.start();
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