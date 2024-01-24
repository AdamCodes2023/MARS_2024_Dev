// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with a single Xbox controller. */
public class SingleHandheldOI implements OperatorInterface {
  private final XboxController controller;

  public SingleHandheldOI(int port) {
    controller = new XboxController(port);
  }

  @Override
  public double getNormalTranslateX() {
    return -controller.getLeftY() * 0.7;
  }

  @Override
  public double getNormalTranslateY() {
    return -controller.getLeftX() * 0.7;
  }

  @Override
  public double getNormalRotate() {
    return controller.getLeftTriggerAxis() * 0.7;
  }

  @Override
  public double getTurboTranslateX() {
    return -controller.getLeftY() * 1.0;
  }

  @Override
  public double getTurboTranslateY() {
    return -controller.getLeftX() * 1.0;
  }

  @Override
  public double getTurboRotate() {
    return controller.getLeftTriggerAxis() * 1.0;
  }

  @Override
  public double getSlowTranslateX() {
    return -controller.getLeftY() * 0.4;
  }

  @Override
  public double getSlowTranslateY() {
    return -controller.getLeftX() * 0.4;
  }

  @Override
  public double getSlowRotate() {
    return controller.getLeftTriggerAxis() * 0.4;
  }

  @Override
  public Trigger getFieldRelativeButton() {
    return new Trigger(controller::getBButton);
  }

  @Override
  public Trigger getResetGyroButton() {
    return new Trigger(controller::getStartButton);
  }

  @Override
  public Trigger getXStanceButton() {
    return new Trigger(controller::getYButton);
  }
}
