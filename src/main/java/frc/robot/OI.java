/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  //Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
  private XboxController controller;
  private XboxController controller1;

  private int layer = 0;

    private OI (){
        controller = new XboxController(0);
        controller1 = new XboxController(1);
    }

    public void updateLayerShift() {
        if (controller.getBumper(GenericHID.Hand.kLeft)) {
            layer = 0;
        } else if (controller.getBumper(GenericHID.Hand.kRight)) {
            layer = 1;
        } else if (controller.getTriggerAxis(GenericHID.Hand.kRight) >= .1) {
            layer = 2;
        }
    }

    public static OI get() {
        if (instance == null) {
            instance = new OI();
        }
        return instance;
    }

    private static OI instance;

    public double getPower() {
        double power;

        power = controller.getY(GenericHID.Hand.kLeft);

        return power;
    }

    public double getTurn() {
        double turn;

        turn = controller.getX(GenericHID.Hand.kRight);

        return turn;
    }

    public double getSecondTankPower() {
        double turn;

        turn = controller.getY(GenericHID.Hand.kRight);

        return turn;
    }

    public boolean getAlign() {
        //return controller.getBButton();

        return false;
    }

    public boolean elevatorHatchLow (){
        return controller1.getAButton();
    }

    public boolean elevatorHatchMid (){
        return controller1.getBButton();
    }

    public boolean elevatorHatchHigh () {
        if (controller1.getTriggerAxis(GenericHID.Hand.kRight) >= .1) {
            return true;
        } else {
            return false;
        }
    }

    public boolean elevatorCargoLow (){
        return controller1.getXButton();
    }

    public boolean elevatorCargoMid (){
        return controller1.getYButton();
    }

    public boolean elevatorCargoHigh (){
        return controller1.getBumper(GenericHID.Hand.kRight);
    }

    public boolean elevatorCargoShip (){
        return controller1.getBumper(GenericHID.Hand.kLeft);
    }

    public boolean elevatorCargoHP () {
        if (controller1.getTriggerAxis(GenericHID.Hand.kLeft) >= 0.1){
           return true;
        } else {
            return false;
        }
    }

    public boolean wristGround () {
        return controller.getAButton();
    }

    public boolean wristStowed () {
        return controller.getYButton();
    }

    public boolean wristHatch () {
        return controller.getXButton();
    }

    public boolean cargoIntake () {
        if (controller.getTriggerAxis(GenericHID.Hand.kLeft) >= 0.1){
           return true;
        } else {
            return false;
        }
    }

    public boolean cargoOutput () {
        if (controller.getTriggerAxis(GenericHID.Hand.kRight) >= 0.1){
            return true;
        } else {
            return false;
        }
    }

    public boolean hatchGrab () {
        return controller.getBumper(GenericHID.Hand.kLeft);
    }

    public boolean placeHatch () {
        return controller.getBumper(GenericHID.Hand.kRight);
    }

    public double testHatch() {
        if (layer == 2) {
            return controller.getY(GenericHID.Hand.kRight);
        } else {
            return 0;
        }
    }

    public double testoCargo() {
        if (layer == 2) {
            return controller.getY(GenericHID.Hand.kLeft);
        } else {
            return 0;
        }
    }

    public double testWrist() {
        if (layer == 1) {
            return controller.getY(GenericHID.Hand.kLeft);
        } else {
            return 0;
        }
    }

    public double testElevator() {
        if (layer == 1) {
            return controller.getY(GenericHID.Hand.kRight);
        } else {
            return 0;
        }
    }

    public double testPower() {
        if (layer == 0) {
            return controller.getY(GenericHID.Hand.kLeft);
        } else {
            return 0;
        }
    }

    public double testTurn() {
        if (layer == 0) {
            return controller.getX(GenericHID.Hand.kRight);
        } else {
            return 0;
        }
    }

    public double sqrInput(double power) {
        if (power > 1.0) {
            power = 1.0;
        } else if (power < -1.0) {
            power = -1.0;
        }

        if (power > 0) {
            return power * power;
        } else if (power < 0) {
            return -(power * power);
        } else {
            return 0;
        }
    }

    public boolean habDeploy() {
        return controller.getStartButton();
    }

    public boolean habRetract() {
        return controller1.getStartButton();
    }

    public boolean habDriveForward() {
        return controller1.getStickButton(GenericHID.Hand.kRight) && ((controller1.getPOV(0) == 1) || (controller1.getPOV(7) == 1) || (controller1.getPOV(1) == 1));
    }

    public boolean habDriveBackwords() {
        return controller1.getStickButton(GenericHID.Hand.kRight) && ((controller1.getPOV(4) == 1) || (controller1.getPOV(3) == 1) || (controller1.getPOV(5) == 1));
    }

    public boolean habLeft() {
        return controller1.getStickButton(GenericHID.Hand.kRight) && ((controller1.getPOV(6) == 1) || (controller1.getPOV(7) == 1) || (controller1.getPOV(5) == 1));
    }

    public boolean habRight() {
        return controller1.getStickButton(GenericHID.Hand.kRight) && ((controller1.getPOV(2) == 1) || (controller1.getPOV(3) == 1) || (controller1.getPOV(1) == 1));
    }
}
