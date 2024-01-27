// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
  private DigitalOutput d1, d2, d3;
  /** Creates a new Lights. */
  public Lights() {
    d1 = new DigitalOutput(1);
    d2 = new DigitalOutput(2);
    d3 = new DigitalOutput(3);
  }

  public void turnOff() {
    d1.set(false);
    d2.set(false);
    d3.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
