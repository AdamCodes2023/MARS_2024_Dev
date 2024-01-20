// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
  DigitalOutput output1;
  DigitalOutput output2;
  public static int initialize = 0;
  // DigitalOutput output3;
  /** Creates a new Lights. */
  public Lights() {
    output1 = new DigitalOutput(0);
    output2 = new DigitalOutput(1);
    // output3 = new DigitalOutput(2);
    if (initialize == 0) {
      turnOffLights();
      initialize++;
    }
  }

  public void turnOffLights() {
    output1.set(true);
    output2.set(true);
    // output3.set(true);
  }

  public void turnYellowLightsOn() {
    output1.set(false);
    output2.set(true);
    // output3.set(true);
  }

  public void turnPurpleLightsOn() {
    output1.set(true);
    output2.set(false);
    // output3.set(true);
  }

  /*
    public void turnBlueLightsOn() {
      output1.set(false);
      output2.set(true);
      //output3.set(false);
    }
  */

  /*
    public void turnRedLightsOn() {
      output1.set(false);
      output2.set(false);
      //output3.set(false);
    }
  */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
