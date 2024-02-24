// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.index;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.lights.Lights;

public class Index extends SubsystemBase {
  private DigitalInput gamePieceProx;
  private boolean lightsTriggered;
  /** Creates a new Index. */
  public Index() {
    gamePieceProx = new DigitalInput(0);

    lightsTriggered = true; 

    createShuffleboard();
  }

  public void createShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("INDEX");
    tab.add("INDEX", this);
    tab.addBoolean("HAVE GAME PIECE", this::hasGamePiece);
  }

  public boolean hasGamePiece() {
    boolean val = !gamePieceProx.get();
    if (val) {
      Lights.getAllianceIntakeLights();
      lightsTriggered = false;
    } else {
      if (!lightsTriggered) {
        Lights.notUsed = true;
        lightsTriggered = true;
      }
    }
    return val;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
