// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Interface for all driver and operator controls. */
public interface OperatorInterface {

  public default double getNormalTranslateX() {
    return 0.0;
  }

  public default double getNormalTranslateY() {
    return 0.0;
  }

  public default double getNormalRotate() {
    return 0.0;
  }

  public default double getTurboTranslateX() {
    return 0.0;
  }

  public default double getTurboTranslateY() {
    return 0.0;
  }

  public default double getTurboRotate() {
    return 0.0;
  }

  public default double getSlowTranslateX() {
    return 0.0;
  }

  public default double getSlowTranslateY() {
    return 0.0;
  }

  public default double getSlowRotate() {
    return 0.0;
  }

  public default double getEmergencyRightRotate() {
    return 0.0;
  }

  public default double getEmergencyLeftRotate() {
    return 0.0;
  }  

  public default Trigger getTurboModeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getSlowModeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getFieldRelativeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getResetGyroButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getXStanceButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getResetPitchButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getEmergencyTurnRightButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getEmergencyTurnLeftButton() {
    return new Trigger(() -> false);
  }    

  public default Trigger getIntakeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getOutakeButton() {
    return new Trigger(() -> false);
  }

  public default double getIntakeSpeed() {
    return 0.0;
  }

  public default double getOutakeSpeed() {
    return 0.0;
  }

  /*
  public default Trigger getPlayMusicButton() {
    return new Trigger(() -> false);
  }
  */

  public default Trigger getAprilTagButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getShootForwardButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getShootReverseButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getPositiveAdjustSpeedButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getNegativeAdjustSpeedButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getPositiveAdjustAngleButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getNegativeAdjustAngleButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getShootAmpButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getShootSpeakerButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getShootSpeakerMidButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getShootSpeakerFarButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getLiftUpButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getLiftDownButton() {
    return new Trigger(() -> false);
  }
}
