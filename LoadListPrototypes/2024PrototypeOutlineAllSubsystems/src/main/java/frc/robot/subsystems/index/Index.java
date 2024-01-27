// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.index;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Index extends SubsystemBase {
  private DigitalInput gamePieceProx;
  /** Creates a new Index. */
  public Index() {
    gamePieceProx = new DigitalInput(0);

    createShuffleboard();
  }

  public void createShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("INDEX");
    tab.add("INDEX", this);
    tab.addBoolean("HAVE GAME PIECE", this::hasGamePiece);
  }

  public boolean hasGamePiece() {
    return !gamePieceProx.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
