// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.shooter.Shooter;

public class StopShootSpeakerAuto extends Command {
  private final Index index;
  private final Shooter shooter;
  private boolean finished;

  /** Creates a new ShootSpeaker. */
  public StopShootSpeakerAuto(Index index, Shooter shooter) {
    this.index = index;
    this.shooter = shooter;

    finished = false;

    addRequirements(index, shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    index.stopIndex();
    shooter.runShooter(0.0);
    finished = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
