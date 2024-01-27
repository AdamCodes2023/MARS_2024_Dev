// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with two Xbox controllers. */
public class DualJoysticksOI implements OperatorInterface {
  private final CommandJoystick translateJoystick;
  private final CommandJoystick rotateJoystick;
  private final Trigger[] translateJoystickButtons;
  private final Trigger[] rotateJoystickButtons;

  public DualJoysticksOI(int translatePort, int rotatePort) {
    translateJoystick = new CommandJoystick(translatePort);
    rotateJoystick = new CommandJoystick(rotatePort);

    // buttons use 1-based indexing such that the index matches the button number; leave index 0 set
    // to null
    this.translateJoystickButtons = new Trigger[13];
    this.rotateJoystickButtons = new Trigger[13];

    for (int i = 1; i < translateJoystickButtons.length; i++) {
      translateJoystickButtons[i] = translateJoystick.button(i);
      rotateJoystickButtons[i] = rotateJoystick.button(i);
    }
  }

  @Override
  public double getNormalTranslateX() {
    return translateJoystick.getY() * 1.0;
  }

  @Override
  public double getNormalTranslateY() {
    return translateJoystick.getX() * 1.0;
  }

  @Override
  public double getNormalRotate() {
    return rotateJoystick.getX() * 1.0;
  }

  @Override
  public double getTurboTranslateX() {
    return translateJoystick.getY() * 1.25;
  }

  @Override
  public double getTurboTranslateY() {
    return translateJoystick.getX() * 1.25;
  }

  @Override
  public double getTurboRotate() {
    return rotateJoystick.getX() * 1.25;
  }

  @Override
  public double getSlowTranslateX() {
    return translateJoystick.getY() * 0.4;
  }

  @Override
  public double getSlowTranslateY() {
    return translateJoystick.getX() * 0.4;
  }

  @Override
  public double getSlowRotate() {
    return rotateJoystick.getX() * 0.4;
  }

  @Override
  public Trigger getTurboModeButton() {
    return translateJoystickButtons[1];
  }

  @Override
  public Trigger getSlowModeButton() {
    return translateJoystickButtons[3];
  }

  @Override
  public Trigger getFieldRelativeButton() {
    return translateJoystickButtons[5];
  }

  @Override
  public Trigger getResetGyroButton() {
    return translateJoystickButtons[9];
  }

  @Override
  public Trigger getXStanceButton() {
    return translateJoystickButtons[4];
  }

  @Override
  public Trigger getResetPitchButton() {
    return translateJoystickButtons[8];
  }

  @Override
  public Trigger getIntakeButton() {
    return translateJoystickButtons[6];
  }

  @Override
  public Trigger getOutakeButton() {
    return translateJoystickButtons[7];
  }

  @Override
  public double getIntakeSpeed() {
    return -((translateJoystick.getZ() * -1 + 1) * 0.5);
  }

  @Override
  public double getOutakeSpeed() {
    return (translateJoystick.getZ() * -1 + 1) * 0.5;
  }

  @Override
  public Trigger getPositiveAngleButton() {
    return rotateJoystickButtons[5];
  }

  @Override
  public Trigger getNegativeAngleButton() {
    return rotateJoystickButtons[6];
  }
}
