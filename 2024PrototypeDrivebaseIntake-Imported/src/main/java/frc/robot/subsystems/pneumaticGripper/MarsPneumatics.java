// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pneumaticGripper;

import edu.wpi.first.wpilibj.Compressor;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Import objects that are needed -> Lightbulb will import automatically
import frc.robot.subsystems.elevator.Elevator;

public class MarsPneumatics extends SubsystemBase {
  // Create subsystem here...

  // PneumaticsControlModule pcm;
  Compressor compressor;
  Solenoid solenoidSingle1, /*solenoidSingle2,*/ solenoidSingle3, solenoidSingle4;
  // DigitalInput readSensor;
  private static int addShuffleboard = 0;
  // private static int stopBar = 0;
  Elevator elevator;

  /** Creates a new Pneumatics. */
  public MarsPneumatics() {
    // pcm = new PneumaticsControlModule(MarsPneumaticsConstants.pneumaticsModuleID);
    compressor =
        new Compressor(
            MarsPneumaticsConstants.pneumaticsModuleID,
            MarsPneumaticsConstants.pneumaticsModuleType);
    solenoidSingle1 =
        new Solenoid(
            MarsPneumaticsConstants.pneumaticsModuleID,
            MarsPneumaticsConstants.pneumaticsModuleType,
            MarsPneumaticsConstants.singleSolenoid1Channel);
    /*
        solenoidSingle2 =
            new Solenoid(
                MarsPneumaticsConstants.pneumaticsModuleID,
                MarsPneumaticsConstants.pneumaticsModuleType,
                MarsPneumaticsConstants.singleSolenoid2Channel);
    */
    solenoidSingle3 =
        new Solenoid(
            MarsPneumaticsConstants.pneumaticsModuleID,
            MarsPneumaticsConstants.pneumaticsModuleType,
            MarsPneumaticsConstants.singleSolenoid3Channel);

    solenoidSingle4 =
        new Solenoid(
            MarsPneumaticsConstants.pneumaticsModuleID,
            MarsPneumaticsConstants.pneumaticsModuleType,
            MarsPneumaticsConstants.singleSolenoid4Channel);

    // readSensor = new DigitalInput(2);
    compressor.enableDigital();
    solenoidSingle1.set(false);
    // solenoidSingle2.set(true);
    solenoidSingle3.set(true);
    solenoidSingle4.set(false);
    // compressor.setClosedLoopControl(true);
    elevator = new Elevator();
    createShuffleboard();
  }

  public void createShuffleboard() {
    if (addShuffleboard < 1) {
      ShuffleboardTab tab = Shuffleboard.getTab("PNEUMATICSTAB");
      tab.add("PNEUMATICS", this);
      tab.addNumber("ANALOG PRESSURE", this::getPressure);
      addShuffleboard++;
    }
  }

  public void solenoidSingle1Set(boolean possibleBoolean1) {
    solenoidSingle1.set(possibleBoolean1);
  }
  /*
    public void solenoidSingle2Set(boolean possibleBoolean2) {
      solenoidSingle2.set(possibleBoolean2);
    }
  */
  public void solenoidSingle3Set(boolean possibleBoolean3) {
    solenoidSingle3.set(possibleBoolean3);
  }

  public void solenoidSingle4Set(boolean possibleBoolean4) {
    solenoidSingle4.set(possibleBoolean4);
  }

  public boolean getSolenoid1() {
    return solenoidSingle1.get();
  }

  public boolean getSolenoid3() {
    return solenoidSingle3.get();
  }

  public boolean getSolenoid4() {
    return solenoidSingle4.get();
  }

  public void grip() {
    solenoidSingle1.set(true);
    // solenoidSingle2.toggle();
  }

  public void unGrip() {
    solenoidSingle1.set(false);
    // solenoidSingle2.toggle();
  }

  public void tripDown() {
    // solenoidSingle3.toggle();
    solenoidSingle3.set(false);
    solenoidSingle4.set(true);
    // stopBar = 0;
  }

  public void tripUp() {
    // solenoidSingle4.toggle();
    solenoidSingle3.set(true);
    solenoidSingle4.set(false);
    // stopBar = 0;
  }

  public void tripStop() {
    solenoidSingle3.set(true);
    solenoidSingle4.set(true);
  }

  // public boolean getReadValue() {
  // return readSensor.get();
  // }

  public double getPressure() {
    return compressor.getPressure();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /*
    if (stopBar == 0) {
      if (getReadValue()) {
        tripStop();
        stopBar = 1;
      }
    }
    */
    if (!solenoidSingle3.get() && solenoidSingle4.get()) {
      elevator.stop();
      // elevator.stopCarriage();
    }
    if (elevator.getElevatorPosition() > 100000) {
      tripUp();
    }
  }
}
