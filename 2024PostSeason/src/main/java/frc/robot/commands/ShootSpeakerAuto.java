// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter_angle.ShooterAngle;

public class ShootSpeakerAuto extends Command {
  private final Intake intake;
  private final Index index;
  private final ShooterAngle shang;
  private final Shooter shooter;
  private final double pos, shooterSpeed;
  private boolean stage1, firstTrigger, finished;

  /** Creates a new ShootSpeaker. */
  public ShootSpeakerAuto(Intake intake, Index index, ShooterAngle shang, Shooter shooter, double pos, double shooterSpeed) {
    this.intake = intake;
    this.index = index;
    this.shang = shang;
    this.shooter = shooter;
    this.pos = pos;
    this.shooterSpeed = shooterSpeed;

    stage1 = true;
    firstTrigger = true;
    finished = false;

    addRequirements(intake, index, shang, shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (stage1) {
      if (firstTrigger) {
        shooter.runShooter(shooterSpeed);
        shang.setPosition(pos);
        firstTrigger = false;
      }
      if (shang.atSetAlignment(pos)) {
        stage1 = false;
        index.runIndex(-1.0);
      }
    }
    if (!stage1) {
      if (!intake.hasGamePiece()) {
        finished = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
