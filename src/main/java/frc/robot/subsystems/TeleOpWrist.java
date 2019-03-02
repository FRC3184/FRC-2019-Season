/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import com.revrobotics.CANSparkMax;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class TeleOpWrist extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    CANSparkMax wristMotor;

    public TeleOpWrist() {
        wristMotor = new CANSparkMax(RobotMap.wrist, CANSparkMaxLowLevel.MotorType.kBrushless);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new TeleopDrive());
    }

    public void wristToPosition(int target) {
        if (wristMotor.getEncoder().getPosition() < target+10 ) {
            wristMotor.set(.25);
        } else if (wristMotor.getEncoder().getPosition() > target-10) {
            wristMotor.set(-.25);
        } else {
            wristMotor.set(0);
        }
    }

    public void test(double power) {
        wristMotor.set(power);
    }
}
