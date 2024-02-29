// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class DriveXMeters extends Command {
  private final Drivetrain drivetrain;
  private final double feet;
  private final boolean reset;
  double ogX;
  boolean finish = false;
  /** Creates a new DriveXMeters. */
  public DriveXMeters(Drivetrain drivetrain, double feet, boolean reset) {
    this.drivetrain = drivetrain;
    this.feet = feet;
    this.reset = reset;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double speed = 1.5;

    if (feet < 0.0) {
      speed = -1.5;
    }

    drivetrain.zeroGyroscope();
    ogX = drivetrain.poseEstimator.getEstimatedPosition().getX();
    drivetrain.drive(speed, 0.0, 0.0);
    // ChassisSpeeds.fromFieldRelativeSpeeds(speed, 0.0, 0.0, drivetrain.getGyroscopeRotation())
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (drivetrain.atXTranslation(feet, ogX, reset) == true) {
      finish = true;
      end(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
