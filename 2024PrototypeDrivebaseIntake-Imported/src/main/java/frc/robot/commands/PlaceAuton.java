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

public class PlaceAuton extends Command {
  private final Drivetrain m_drivetrainSubsystem;
  private final Elevator m_elevator;
  private final ElevatorCarriage m_elevatorCarriage;
  private final GripperWrist m_gripperWrist;
  private final MarsPneumatics m_marsPneumatics;
  boolean stage1 = false;
  boolean stage2 = false;
  boolean stage3 = false;
  boolean stage4 = false;
  boolean stage5 = false;
  /** Creates a new PlaceAuton. */
  public PlaceAuton(
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
    m_marsPneumatics.tripUp();
    m_gripperWrist.runUntilReverseLimit();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (stage1 == false) {
      if (m_gripperWrist.returnReverseLimitSwitchState() == 1) {
        m_marsPneumatics.grip();
        stage1 = true;
      }
    }

    if (stage1 && stage2 == false) {
      if (m_marsPneumatics.getSolenoid1()) {
        m_elevator.runToLowScorePosition();
        m_elevatorCarriage.runToLowScorePosition();
        m_gripperWrist.runToElevatorSamePos();
        stage2 = true;
      }
    }

    if (stage1 && stage2 && stage3 == false) {
      if (m_elevator.getElevatorPosition() <= 180000 && m_elevator.getElevatorPosition() >= 170000) {
        m_marsPneumatics.unGrip();
        stage3 = true;
      }
    }

    if (stage1 && stage2 && stage3 && stage4 == false) {
      if (!m_marsPneumatics.getSolenoid1()) {
        m_elevator.runToTravelPosition();
        m_elevatorCarriage.runToTravelPosition();
        m_gripperWrist.runToElevatorSamePos();
        stage4 = true;        
      }
    }
    
    if (stage1 && stage2 && stage3 && stage4 && stage5 == false) {
      if (m_elevator.getElevatorPosition() <= 5000 && m_elevator.getElevatorPosition() >= -5000) {
        stage5 = true;
      }
    }

    if (stage1 && stage2 && stage3 && stage4 && stage5) {
      end(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
