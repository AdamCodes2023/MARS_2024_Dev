// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lights;

import edu.wpi.first.hal.I2CJNI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.nio.ByteBuffer;

public class Lights extends SubsystemBase {
  private byte arduinoAddress = 0;
  private byte bytesToSend = 1;
  /** Creates a new Lights. */
  public Lights() {
    I2CJNI.i2CInitialize(1);
  }

  public void turnOff() {
    byte[] dataBytes = {1};
    ByteBuffer dataBuffer = ByteBuffer.wrap(dataBytes);
    I2CJNI.i2CWrite(1, arduinoAddress, dataBuffer, bytesToSend);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
