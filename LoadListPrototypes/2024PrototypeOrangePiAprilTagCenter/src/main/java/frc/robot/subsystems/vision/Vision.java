// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;

public class Vision extends SubsystemBase {
  private final double CAMERA_HEIGHT_METERS;
  private final double CAMERA_PITCH_RADIANS;
  private PhotonCamera camera;
  private PIDController rotationPID;

  /** Creates a new Vision. */
  public Vision() {
    // Constants such as camera and target height stored. Change per robot and goal!
    CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    // Angle between horizontal and the camera.
    CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
    // Change this to match the name of your camera.
    camera = new PhotonCamera("OrangePi");
    // Rotation PID Controller
    rotationPID = new PIDController(.08, .05, .002);
    rotationPID.setTolerance(.25); // allowable angle error
    rotationPID.enableContinuousInput(
        0, 360); // it is faster to go 1 degree from 359 to 0 instead of 359 degrees
    rotationPID.setSetpoint(0); // 0 = apriltag angle
    // Shuffleboard Tab.
    createShuffleboard();
  }

  public void createShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("VISION");
    //tab.add("VISION", this);
    //tab.addNumber("ANGLE_RECORRECTION", this::aprilTagLock);
  }

  public double aprilTagLock() {
    var result = camera.getLatestResult();
    if (result.hasTargets()) {
      return -1.0 * rotationPID.calculate(result.getBestTarget().getYaw());
    } else {
      return 0.0;
    }
  }

  public double centerOnAprilTag() {
    // Vision-alignment mode
    // Query the latest result from PhotonVision
    double rotationSpeed;
    var result = camera.getLatestResult();

    if (result.hasTargets()) {
      // Calculate angular turn power
      if (result.getBestTarget().getYaw() < -3) {
        rotationSpeed = -0.2;
      } else if (result.getBestTarget().getYaw() > 3) {
        rotationSpeed = 0.2;
      } else {
        rotationSpeed = 0.0;
      }
    } else {
      // If we have no targets, stay still.
      rotationSpeed = 0;
    }
    return rotationSpeed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
