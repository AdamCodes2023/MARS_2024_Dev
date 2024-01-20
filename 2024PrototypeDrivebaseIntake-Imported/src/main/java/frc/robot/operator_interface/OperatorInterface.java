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

  public default Trigger getCenterOnChargingPadButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getResetPitchButton() {
    return new Trigger(() -> false);
  }

  // public default Trigger getCenterOnLowNodeButton() {
  // return new Trigger(() -> false);
  // }

  public default Trigger getCenterOnMidNodeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getCenterOnHighNodeButton() {
    return new Trigger(() -> false);
  }

  // public default Trigger getCenterOnConeButton() {
  // return new Trigger(() -> false);
  // }

  // public default Trigger getCenterOnCubeButton() {
  // return new Trigger(() -> false);
  // }

  public default Trigger getCenterOnObjectButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getCenterOnAprilTagButton() {
    return new Trigger(() -> false);
  }
}
