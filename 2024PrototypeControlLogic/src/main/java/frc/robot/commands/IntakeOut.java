// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter_angle.ShooterAngle;

public class IntakeOut extends Command {
  private final Index index;
  private final Intake intake;
  private final ShooterAngle shang;

  /** Creates a new IntakeOut. */
  public IntakeOut(Index index, Intake intake, ShooterAngle shang) {
    this.index = index;
    this.intake = intake;
    this.shang = shang;
    addRequirements(index, intake, shang);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    boolean firstTrigger = true;
    while (!shang.atIntakeAlignment()) {
      if (firstTrigger) {
        shang.intakePosition();
        firstTrigger = false;
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    index.runIndex(-0.7);
    intake.runIntake(0.7);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    boolean firstTrigger = true;
    index.stopIndex();
    intake.stopIntake();
    if (!intake.hasGamePiece()) {
      while (!shang.atHardStopBottom()) {
        if (firstTrigger) {
          shang.stopMotor();
          firstTrigger = false;
        }
      }
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
