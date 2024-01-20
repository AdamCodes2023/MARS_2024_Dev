// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.team3061.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

public class VisionTape extends SubsystemBase {
  private PhotonCamera camera;
  // Constants such as camera and target height stored. Change per robot and goal!
  // FIXME: Set the correct height of the camera on the robot.
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(25);
  // Angle between horizontal and the camera.
  // FIXME: Set the correct angle of the camera on the robot.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(90);

  public boolean atPosDistance = false;
  public boolean atPosAngle = false;

  final double LINEAR_P = 0.10;
  final double LINEAR_D = 0.00;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  final double ANGULAR_P = 0.10;
  final double ANGULAR_D = 0.00;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  PIDController aprilTurnController = new PIDController(0.05, 0, 0);

  /** Creates a new VisionTape. */
  public VisionTape(PhotonCamera camera) {
    this.camera = camera;
    forwardController.setTolerance(0.0);
    turnController.setTolerance(0.0);
    aprilTurnController.setTolerance(0.0);
  }

  // FIXME: Set the correct height of targets and proper distance away for scoring game pieces.
  // FIXME: Set the proper threshold for centered on target.

  public double calculateDistanceToLowTarget() {
    if (camera.getPipelineIndex() != 1) {
      camera.setPipelineIndex(1);
      WaitCommand wait = new WaitCommand(2.0);
      wait.schedule();
    }
    double TARGET_HEIGHT_METERS = Units.feetToMeters(2.291667);
    double GOAL_RANGE_METERS = Units.feetToMeters(4.5);
    double forwardSpeed;
    // Vision-alignment mode
    // Query the latest result from PhotonVision
    var result = camera.getLatestResult();
    if (result.hasTargets()) {
      // First calculate range
      double range =
          PhotonUtils.calculateDistanceToTargetMeters(
              CAMERA_HEIGHT_METERS,
              TARGET_HEIGHT_METERS,
              CAMERA_PITCH_RADIANS,
              Units.degreesToRadians(result.getBestTarget().getPitch()));

      // Use this range as the measurement we give to the PID controller.
      // -1.0 required to ensure positive PID controller effort _increases_ range
      // SmartDashboard.putNumber("range", range);
      forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);
      // SmartDashboard.putNumber("preForward", forwardSpeed);
    } else {
      // If we have no targets, stay still.
      forwardSpeed = 0;
    }
    forwardSpeed *= 10;
    if (forwardSpeed < 0.1 && forwardSpeed > -0.1) {
      forwardSpeed = 0.0;
      atPosDistance = true;
    }
    // SmartDashboard.putNumber("postForward", forwardSpeed);
    return forwardSpeed;
  }

  public double calculateAngleToLowTarget() {
    if (camera.getPipelineIndex() != 1) {
      camera.setPipelineIndex(1);
      WaitCommand wait = new WaitCommand(2.0);
      wait.schedule();
    }
    double rotationSpeed;
    // Vision-alignment mode
    // Query the latest result from PhotonVision
    var result = camera.getLatestResult();

    if (result.hasTargets()) {
      // Calculate angular turn power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
    } else {
      // If we have no targets, stay still.
      rotationSpeed = 0.0;
    }
    if (rotationSpeed < 0.2 && rotationSpeed > -0.2) {
      rotationSpeed = 0.0;
      atPosAngle = true;
    }
    return -rotationSpeed;
  }

  public double calculateDistanceToMidTarget() {
    if (camera.getPipelineIndex() != 1) {
      camera.setPipelineIndex(1);
      WaitCommand wait = new WaitCommand(2.0);
      wait.schedule();
    }
    double TARGET_HEIGHT_METERS = Units.feetToMeters(2.00131);
    // double GOAL_RANGE_METERS = Units.feetToMeters(1.90289);
    double GOAL_RANGE_METERS = Units.feetToMeters(2.20289);
    double forwardSpeed;
    // Vision-alignment mode
    // Query the latest result from PhotonVision
    var result = camera.getLatestResult();
    if (result.hasTargets()) {
      // First calculate range
      double range =
          PhotonUtils.calculateDistanceToTargetMeters(
              CAMERA_HEIGHT_METERS,
              TARGET_HEIGHT_METERS,
              CAMERA_PITCH_RADIANS,
              Units.degreesToRadians(result.getBestTarget().getPitch()));

      // Use this range as the measurement we give to the PID controller.
      // -1.0 required to ensure positive PID controller effort _increases_ range
      // SmartDashboard.putNumber("range", range);
      forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);
      // SmartDashboard.putNumber("preForward", forwardSpeed);
    } else {
      // If we have no targets, stay still.
      forwardSpeed = 0;
    }
    forwardSpeed *= 10;
    if (forwardSpeed < 0.1 && forwardSpeed > -0.1) {
      forwardSpeed = 0.0;
      atPosDistance = true;
    }
    // SmartDashboard.putNumber("postForward", forwardSpeed);
    return forwardSpeed;
  }

  public double calculateAngleToMidTarget() {
    if (camera.getPipelineIndex() != 1) {
      camera.setPipelineIndex(1);
      WaitCommand wait = new WaitCommand(2.0);
      wait.schedule();
    }
    double rotationSpeed;
    // Vision-alignment mode
    // Query the latest result from PhotonVision
    var result = camera.getLatestResult();

    if (result.hasTargets()) {
      // Calculate angular turn power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
    } else {
      // If we have no targets, stay still.
      rotationSpeed = 0.0;
    }
    if (rotationSpeed < 0.2 && rotationSpeed > -0.2) {
      rotationSpeed = 0.0;
      atPosAngle = true;
    }
    return -rotationSpeed;
  }

  public double calculateDistanceToHighTarget() {
    if (camera.getPipelineIndex() != 2) {
      camera.setPipelineIndex(2);
      WaitCommand wait = new WaitCommand(2.0);
      wait.schedule();
    }
    double TARGET_HEIGHT_METERS = Units.feetToMeters(3.64173);
    // double GOAL_RANGE_METERS = Units.feetToMeters(3.31365);
    double GOAL_RANGE_METERS = Units.feetToMeters(3.61365);
    double forwardSpeed;
    // Vision-alignment mode
    // Query the latest result from PhotonVision
    var result = camera.getLatestResult();
    if (result.hasTargets()) {
      // First calculate range
      double range =
          PhotonUtils.calculateDistanceToTargetMeters(
              CAMERA_HEIGHT_METERS,
              TARGET_HEIGHT_METERS,
              CAMERA_PITCH_RADIANS,
              Units.degreesToRadians(result.getBestTarget().getPitch()));

      // Use this range as the measurement we give to the PID controller.
      // -1.0 required to ensure positive PID controller effort _increases_ range
      // SmartDashboard.putNumber("range", range);
      forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);
      // SmartDashboard.putNumber("preForward", forwardSpeed);
    } else {
      // If we have no targets, stay still.
      forwardSpeed = 0;
    }
    forwardSpeed *= 10;
    if (forwardSpeed < 0.1 && forwardSpeed > -0.1) {
      forwardSpeed = 0.0;
      atPosDistance = true;
    }
    // SmartDashboard.putNumber("postForward", forwardSpeed);
    return forwardSpeed;
  }

  public double calculateAngleToHighTarget() {
    if (camera.getPipelineIndex() != 2) {
      camera.setPipelineIndex(2);
      WaitCommand wait = new WaitCommand(2.0);
      wait.schedule();
    }
    double rotationSpeed;
    // Vision-alignment mode
    // Query the latest result from PhotonVision
    var result = camera.getLatestResult();

    if (result.hasTargets()) {
      // Calculate angular turn power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
    } else {
      // If we have no targets, stay still.
      rotationSpeed = 0.0;
    }
    if (rotationSpeed < 0.2 && rotationSpeed > -0.2) {
      rotationSpeed = 0.0;
      atPosAngle = true;
    }
    return -rotationSpeed;
  }

  public double calculateDistanceToAprilTag() {
    if (camera.getPipelineIndex() != 0) {
      camera.setPipelineIndex(0);
      WaitCommand wait = new WaitCommand(2.0);
      wait.schedule();
    }
    double TARGET_HEIGHT_METERS = Units.feetToMeters(2.181759);
    double GOAL_RANGE_METERS = Units.feetToMeters(4.5);
    double forwardSpeed;
    // Vision-alignment mode
    // Query the latest result from PhotonVision
    var result = camera.getLatestResult();
    if (result.hasTargets()) {
      // First calculate range
      double range =
          PhotonUtils.calculateDistanceToTargetMeters(
              CAMERA_HEIGHT_METERS,
              TARGET_HEIGHT_METERS,
              CAMERA_PITCH_RADIANS,
              Units.degreesToRadians(result.getBestTarget().getPitch()));

      // Use this range as the measurement we give to the PID controller.
      // -1.0 required to ensure positive PID controller effort _increases_ range
      // SmartDashboard.putNumber("range", range);
      forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);
      // SmartDashboard.putNumber("preForward", forwardSpeed);
    } else {
      // If we have no targets, stay still.
      forwardSpeed = 0;
    }
    forwardSpeed *= 10;
    if (forwardSpeed < 0.1 && forwardSpeed > -0.1) {
      forwardSpeed = 0.0;
      atPosDistance = true;
    }
    // SmartDashboard.putNumber("postForward", forwardSpeed);
    return forwardSpeed;
  }

  public double calculateAngleToAprilTag() {
    if (camera.getPipelineIndex() != 0) {
      camera.setPipelineIndex(0);
      WaitCommand wait = new WaitCommand(2.0);
      wait.schedule();
    }
    double rotationSpeed;
    // Vision-alignment mode
    // Query the latest result from PhotonVision
    var result = camera.getLatestResult();

    if (result.hasTargets()) {
      // Calculate angular turn power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      rotationSpeed = -aprilTurnController.calculate(result.getBestTarget().getYaw(), 0);
    } else {
      // If we have no targets, stay still.
      rotationSpeed = 0.0;
    }
    if (rotationSpeed < 0.20 && rotationSpeed > -0.20) {
      rotationSpeed = 0.0;
      atPosAngle = true;
    }
    return -rotationSpeed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
