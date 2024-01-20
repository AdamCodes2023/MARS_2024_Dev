// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.team3061.vision;

import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonUtils;

public class VisionConeCube extends SubsystemBase {
  private NetworkTable camera;
  private NetworkTableEntry objectCords;
  public boolean atPosDistance = false;
  public boolean atPosAngle = false;
  // Constants such as camera and target height stored. Change per robot and goal!
  // FIXME: Set the correct height of the camera on the robot.
  // final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
  // Angle between horizontal and the camera.
  // FIXME: Set the correct angle of the camera on the robot.
  // final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  final double LINEAR_P = 0.10;
  final double LINEAR_D = 0.00;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  final double ANGULAR_P = 0.10;
  final double ANGULAR_D = 0.00;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  /** Creates a new VisionConeCube. */
  public VisionConeCube() {
    camera = NetworkTableInstance.getDefault().getTable(VisionConstants.CONE_CUBE_CAMERA_NAME);
    objectCords = camera.getEntry("objects");
  }

  public int closestObject() {
    camera = NetworkTableInstance.getDefault().getTable(VisionConstants.CONE_CUBE_CAMERA_NAME);
    objectCords = camera.getEntry("objects");
    String[] analyzeCords = objectCords.getStringArray(new String[0]);
    int index = 0;
    if (analyzeCords.length >= 2) {
      double targetxvalue =
          Double.parseDouble(analyzeCords[0].substring(0, analyzeCords[0].indexOf(",")));
      double targetDifference = Math.abs(0.0 - targetxvalue);

      double targetyvalue =
          Double.parseDouble(analyzeCords[0].substring(analyzeCords[0].indexOf(",") + 1));
      double targetVertical = Math.abs(0.0 - targetyvalue);

      for (int i = 0; i < analyzeCords.length; i++) {
        double xvalue =
            Double.parseDouble(analyzeCords[i].substring(0, analyzeCords[i].indexOf(",")));
        double difference = Math.abs(0.0 - xvalue);

        double yvalue =
            Double.parseDouble(analyzeCords[i].substring(analyzeCords[i].indexOf(",") + 1));
        double vertical = Math.abs(0.0 - yvalue);
        if (targetDifference > difference && targetVertical > vertical) {
          targetDifference = difference;
          targetVertical = vertical;
          index = i;
        }
      }
    }
    if (analyzeCords.length > 0) {
      return index;
    } else {
      return -1;
    }
  }

  public double getx(int index) {
    camera = NetworkTableInstance.getDefault().getTable(VisionConstants.CONE_CUBE_CAMERA_NAME);
    objectCords = camera.getEntry("objects");
    String[] analyzeCords = objectCords.getStringArray(new String[0]);
    if (index >= 0 && analyzeCords.length > 0) {
      double targetxvalue =
          Double.parseDouble(analyzeCords[index].substring(0, analyzeCords[index].indexOf(",")));
      SmartDashboard.putNumber("X Coordinate", targetxvalue);
      return targetxvalue - VisionConstants.MACHINE_VISION_OFFSET;
    } else {
      SmartDashboard.putNumber("X Coordinate", 0.0);
      return 0.0;
    }
  }

  public double gety(int index) {
    camera = NetworkTableInstance.getDefault().getTable(VisionConstants.CONE_CUBE_CAMERA_NAME);
    objectCords = camera.getEntry("objects");
    String[] analyzeCords = objectCords.getStringArray(new String[0]);
    if (index >= 0 && analyzeCords.length > 0) {
      double targetyvalue =
          Double.parseDouble(analyzeCords[index].substring(analyzeCords[index].indexOf(",") + 1));
      SmartDashboard.putNumber("Y Coordinate", targetyvalue);
      return targetyvalue;
    } else {
      SmartDashboard.putNumber("Y Coordinate", 0.0);
      return 0.0;
    }
  }

  // FIXME: Set the correct height of targets and proper distance away for scoring game pieces.
  // FIXME: Set the proper threshold for centered on target.

  public double calculateDistanceToCube() {
    camera = NetworkTableInstance.getDefault().getTable(VisionConstants.CONE_CUBE_CAMERA_NAME);
    objectCords = camera.getEntry("objects");
    String[] analyzeCords = objectCords.getStringArray(new String[0]);
    double GOAL_RANGE_INCHES = 36;
    double forwardSpeed;
    // Vision-alignment mode

    if (analyzeCords.length > 0) {
      // First calculate range
      double current_distance = calculateDistanceFromLimelightToGoal(true);

      // Use this range as the measurement we give to the PID controller.
      // -1.0 required to ensure positive PID controller effort _increases_ range
      forwardSpeed = -forwardController.calculate(current_distance, GOAL_RANGE_INCHES);
    } else {
      // If we have no targets, stay still.
      forwardSpeed = 0;
    }
    if (forwardSpeed < 0.2 && forwardSpeed > -0.2) {
      forwardSpeed = 0.0;
      atPosDistance = true;
    }
    return forwardSpeed;
  }

  public double calculateAngleToCube() {
    camera = NetworkTableInstance.getDefault().getTable(VisionConstants.CONE_CUBE_CAMERA_NAME);
    objectCords = camera.getEntry("objects");
    String[] analyzeCords = objectCords.getStringArray(new String[0]);
    double rotationSpeed;
    // Vision-alignment mode

    if (analyzeCords.length > 0) {
      // Calculate angular turn power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      rotationSpeed = -turnController.calculate(getx(closestObject()), 0);
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

  public double calculateDistanceToCone() {
    camera = NetworkTableInstance.getDefault().getTable(VisionConstants.CONE_CUBE_CAMERA_NAME);
    objectCords = camera.getEntry("objects");
    String[] analyzeCords = objectCords.getStringArray(new String[0]);
    double GOAL_RANGE_INCHES = 36;
    double forwardSpeed;
    // Vision-alignment mode

    if (analyzeCords.length > 0) {
      // First calculate range
      double current_distance = calculateDistanceFromLimelightToGoal(false);

      // Use this range as the measurement we give to the PID controller.
      // -1.0 required to ensure positive PID controller effort _increases_ range
      forwardSpeed = -forwardController.calculate(current_distance, GOAL_RANGE_INCHES);
    } else {
      // If we have no targets, stay still.
      forwardSpeed = 0;
    }
    if (forwardSpeed < 0.2 && forwardSpeed > -0.2) {
      forwardSpeed = 0.0;
      atPosDistance = true;
    }
    return forwardSpeed;
  }

  public double calculateAngleToCone() {
    camera = NetworkTableInstance.getDefault().getTable(VisionConstants.CONE_CUBE_CAMERA_NAME);
    objectCords = camera.getEntry("objects");
    String[] analyzeCords = objectCords.getStringArray(new String[0]);
    double rotationSpeed;
    // Vision-alignment mode

    if (analyzeCords.length > 0) {
      // Calculate angular turn power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      rotationSpeed = -turnController.calculate(getx(closestObject()), 0);
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

  public double calculateDistanceToObject() {
    camera = NetworkTableInstance.getDefault().getTable(VisionConstants.CONE_CUBE_CAMERA_NAME);
    objectCords = camera.getEntry("objects");
    String[] analyzeCords = objectCords.getStringArray(new String[0]);
    double GOAL_RANGE_INCHES = 36;
    double forwardSpeed;
    // Vision-alignment mode

    if (analyzeCords.length > 0) {
      // First calculate range
      double current_distance = 0.0;
      // if (tn.getString("NONE").equals("cone")) {
      // current_distance = calculateDistanceFromLimelightToGoal(false);
      // } else {
      current_distance = calculateDistanceFromLimelightToGoal(true);
      // }
      // Use this range as the measurement we give to the PID controller.
      // -1.0 required to ensure positive PID controller effort _increases_ range
      forwardSpeed = -forwardController.calculate(current_distance, GOAL_RANGE_INCHES);
    } else {
      // If we have no targets, stay still.
      forwardSpeed = 0;
    }
    if (forwardSpeed < 0.2 && forwardSpeed > -0.2) {
      forwardSpeed = 0.0;
      atPosDistance = true;
    }
    return forwardSpeed;
  }

  public double calculateAngleToObject() {
    camera = NetworkTableInstance.getDefault().getTable(VisionConstants.CONE_CUBE_CAMERA_NAME);
    objectCords = camera.getEntry("objects");
    String[] analyzeCords = objectCords.getStringArray(new String[0]);
    double rotationSpeed;
    // Vision-alignment mode

    if (analyzeCords.length > 0) {
      // Calculate angular turn power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      rotationSpeed = -turnController.calculate(getx(closestObject()), 0);
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

  public double calculateDistanceFromLimelightToGoal(boolean isCube) {
    camera = NetworkTableInstance.getDefault().getTable(VisionConstants.CONE_CUBE_CAMERA_NAME);
    objectCords = camera.getEntry("objects");
    double targetOffsetAngle_Vertical = gety(closestObject());
    targetOffsetAngle_Vertical *=
        (36.955 / 720.0); // FIXME: Identify FOV of camera and degrees per pixel.

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 0.0;

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 25.0;

    // distance from the target to the floor
    double goalHeightInches = 0.0;
    if (isCube) {
      goalHeightInches = 2.5;
    } else {
      goalHeightInches = 2.5;
    }

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    // calculate distance
    double distanceFromLimelightToGoalInches =
        (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    return distanceFromLimelightToGoalInches;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // getx(closestObject());
    // gety(closestObject());
  }
}
