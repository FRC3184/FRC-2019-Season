/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SPI;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // For example to map the left and right motors, you could define the
    // following variables to use with your drivetrain subsystem.
    public static int leftDriveMaster= 0;
    public static int rightDriveMaster = 1;
    public static int leftDriveSlave = 0;
    public static int rightDriveSlave = 1;

    public static int elevatorMaster = 2;
    public static int elevatorSlave = 3;

    public static int cargoIntake = 0;
    public static int hatchIntake = 4;

    public static int wrist = 1;
    public static int habFlipper = 2;
    public static int habLeftWheel = 3; // from robots normal perspective
    public static int habRightWheel = 4; // from robots normal perspective

    public static SPI.Port gyroPort = SPI.Port.kMXP;

    public static int wristLimitSwitchReverse = 0;
    public static int wristLimitSwitchForward = 1;
    public static int elevatorLimitSwitchReverse = 2;
    public static int elevatorLimitSwitchForward = 3;
    public static int hatchLimitSwitchReverse = 4;
    public static int hatchLimitSwitchForward = 5;
    public static int habFlipperLimitSwitchReverse = 6;
    public static int habFlipperLimitSwitchForward = 7;

    // If you are using multiple modules, make sure to define both the port
    // number and the module. For example you with a rangefinder:
    // public static int rangefinderPort = 1;
    // public static int rangefinderModule = 1;
}
