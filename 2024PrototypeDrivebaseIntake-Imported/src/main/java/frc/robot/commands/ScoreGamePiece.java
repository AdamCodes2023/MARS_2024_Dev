// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorCarriage;
import frc.robot.subsystems.pneumaticGripper.GripperWrist;
import frc.robot.subsystems.pneumaticGripper.MarsPneumatics;

public class ScoreGamePiece extends Command {
  private final Drivetrain m_drivetrainSubsystem;
  private final Elevator m_elevator;
  private final ElevatorCarriage m_elevatorCarriage;
  private final GripperWrist m_gripperWrist;
  private final MarsPneumatics m_marsPneumatics;
  /** Creates a new ScoreGamePiece. */
  public ScoreGamePiece(
      Drivetrain drivetrainSubsystem,
      Elevator elevator,
      ElevatorCarriage elevatorCarriage,
      GripperWrist gripperWrist,
      MarsPneumatics marsPneumatics) {
    this.m_drivetrainSubsystem = drivetrainSubsystem;
    this.m_elevator = elevator;
    this.m_elevatorCarriage = elevatorCarriage;
    this.m_gripperWrist = gripperWrist;
    this.m_marsPneumatics = marsPneumatics;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrainSubsystem.stop();
    // m_elevator.runUntilForwardLimit();
    // m_elevator.runToLowScorePosition();
    m_elevatorCarriage.runToTravelPosition();
    m_gripperWrist.runToElevatorSamePos();
    // WaitCommand wait = new WaitCommand(1.0);
    // wait.schedule();
    m_elevator.runToTravelPosition();
    m_drivetrainSubsystem.visionTapeCalc.atPosDistance = false;
    m_drivetrainSubsystem.visionTapeCalc.atPosAngle = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_drivetrainSubsystem.visionTapeCalc.atPosDistance
        && m_drivetrainSubsystem.visionTapeCalc.atPosAngle) {
      m_drivetrainSubsystem.stop();
      m_elevatorCarriage.runToLowScorePosition();
      m_gripperWrist.runToElevatorSamePos();
      // WaitCommand wait = new WaitCommand(1.0);
      // wait.schedule();
      m_elevator.runToLowScorePosition();
      // WaitCommand anotherWait = new WaitCommand(1.0);
      // anotherWait.schedule();
      m_marsPneumatics.unGrip();
      cancel();
    } else {
      m_drivetrainSubsystem.centerOnMidNode();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.visionTapeCalc.atPosDistance = false;
    m_drivetrainSubsystem.visionTapeCalc.atPosAngle = false;
    m_drivetrainSubsystem.stop();
    m_elevator.runToTravelPosition();
    // WaitCommand wait = new WaitCommand(1.0);
    // wait.schedule();
    m_elevatorCarriage.runToTravelPosition();
    m_gripperWrist.runToElevatorSamePos();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
