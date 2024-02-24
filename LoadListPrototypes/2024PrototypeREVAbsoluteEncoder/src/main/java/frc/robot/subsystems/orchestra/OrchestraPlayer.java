// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.orchestra;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;

public class OrchestraPlayer extends SubsystemBase {
  private Orchestra music;
  private TalonFX instrument;
  /** Creates a new OrchestraPlayer. */
  public OrchestraPlayer() {
    instrument = new TalonFX(66);
    ArrayList<TalonFX> instruments = new ArrayList<TalonFX>();
    instruments.add(instrument);
    music = new Orchestra(instruments, "ImperialMarch.chrp");
  }

  public void playMusic() {
    music.play();
  }

  public void pauseMusic() {
    music.pause();
  }

  public void stopMusic() {
    music.stop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
