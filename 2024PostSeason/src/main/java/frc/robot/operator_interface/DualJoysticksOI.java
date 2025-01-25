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
  private final CommandJoystick extraButtonJoystick;
  private final Trigger[] translateJoystickButtons;
  private final Trigger[] rotateJoystickButtons;
  private final Trigger[] extraJoystickButtons;

  public DualJoysticksOI(int translatePort, int rotatePort, int extraPort) {
    translateJoystick = new CommandJoystick(translatePort);
    rotateJoystick = new CommandJoystick(rotatePort);
    extraButtonJoystick = new CommandJoystick(extraPort);

    // buttons use 1-based indexing such that the index matches the button number; leave index 0 set
    // to null
    this.translateJoystickButtons = new Trigger[13];
    this.rotateJoystickButtons = new Trigger[13];
    this.extraJoystickButtons = new Trigger[13];

    for (int i = 1; i < translateJoystickButtons.length; i++) {
      translateJoystickButtons[i] = translateJoystick.button(i);
      rotateJoystickButtons[i] = rotateJoystick.button(i);
      extraJoystickButtons[i] = extraButtonJoystick.button(i);
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
  public double getEmergencyRightRotate() {
    return 0.75;
  }

  @Override
  public double getEmergencyLeftRotate() {
    return -0.75;
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
    return translateJoystickButtons[10];
  }

  @Override
  public Trigger getResetPitchButton() {
    return translateJoystickButtons[8];
  }

  @Override
  public Trigger getEmergencyTurnRightButton() {
    return translateJoystickButtons[5];
  }

  @Override
  public Trigger getEmergencyTurnLeftButton() {
    return translateJoystickButtons[4];
  }

  @Override
  public double getIntakeSpeed() {
    return -((translateJoystick.getZ() * -1 + 1) * 0.5);
  }

  @Override
  public double getOutakeSpeed() {
    return (translateJoystick.getZ() * -1 + 1) * 0.5;
  }

  /*
  @Override
  public Trigger getPlayMusicButton() {
    return rotateJoystickButtons[4];
  }
  */

  @Override
  public Trigger getAprilTagButton() {
    return translateJoystickButtons[6];
  }

  @Override
  public Trigger getShootAmpButton() {
    return rotateJoystickButtons[1];
  }

  @Override
  public Trigger getShootSpeakerButton() {
    return rotateJoystickButtons[2];
  }

  @Override
  public Trigger getShootSpeakerMidButton() {
    return rotateJoystickButtons[3];
  }

  @Override
  public Trigger getShootSpeakerFarButton() {
    return rotateJoystickButtons[4];
  }

  @Override
  public Trigger getShootForwardButton() {
    return extraJoystickButtons[1];
  }

  @Override
  public Trigger getPositiveAdjustSpeedButton() {
    return rotateJoystickButtons[8];
  }

  @Override
  public Trigger getNegativeAdjustSpeedButton() {
    return rotateJoystickButtons[9];
  }

  @Override
  public Trigger getPositiveAdjustAngleButton() {
    return rotateJoystickButtons[7];
  }

  @Override
  public Trigger getNegativeAdjustAngleButton() {
    return extraJoystickButtons[2];
  }

  @Override
  public Trigger getShootReverseButton() {
    return rotateJoystickButtons[10];
  }

  @Override
  public Trigger getIntakeButton() {
    return rotateJoystickButtons[5];
  }

  @Override
  public Trigger getOutakeButton() {
    return rotateJoystickButtons[6];
  }

  @Override
  public Trigger getLiftUpButton() {
    return rotateJoystickButtons[11];
  }

  @Override
  public Trigger getLiftDownButton() {
    return rotateJoystickButtons[12];
  }
}
