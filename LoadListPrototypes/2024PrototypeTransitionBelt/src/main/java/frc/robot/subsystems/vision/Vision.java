// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;

public class Vision extends SubsystemBase {
  private final double CAMERA_HEIGHT_METERS;
  private final double CAMERA_PITCH_RADIANS;
  private PhotonCamera camera;

  /** Creates a new Vision. */
  public Vision() {
    // Constants such as camera and target height stored. Change per robot and goal!
    CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    // Angle between horizontal and the camera.
    CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
    // Change this to match the name of your camera.
    camera = new PhotonCamera("Limelight");
    // Shuffleboard Tab.
    createShuffleboard();
  }

  public void createShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("VISION");
    tab.add("VISION", this);
    tab.addNumber("ANGLE_RECORRECTION", this::centerOnAprilTag);
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
