// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.lights.Lights;

public class Intake extends SubsystemBase {
  private DigitalInput gamePieceProx;
  private boolean lightsTriggered;
  private CANSparkFlex intakeMotor;
  // private TalonSRX transitionBelt;
  /** Creates a new Intake. */
  public Intake() {
    lightsTriggered = true;
    gamePieceProx = new DigitalInput(0);

    intakeMotor = new CANSparkFlex(32, MotorType.kBrushless);
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setIdleMode(IdleMode.kCoast);

    // transitionBelt = new TalonSRX(59);
    // transitionBelt.configFactoryDefault();
    // transitionBelt.setNeutralMode(NeutralMode.Coast);

    createShuffleboard();
  }

  public void createShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("INTAKE");
    tab.add("INTAKE", this);
    tab.addNumber("INTAKE SPEED (PERCENTAGE)", this::getIntakeSpeed);
    tab.addBoolean("HAVE GAME PIECE", this::hasGamePiece);
  }

  public double getIntakeSpeed() {
    return intakeMotor.get() * 100;
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
    // transitionBelt.set(TalonSRXControlMode.PercentOutput, 0.0);
  }

  public void runIntake(double speed) {
    intakeMotor.set(speed);
    // transitionBelt.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public boolean hasGamePiece() {
    boolean val = !gamePieceProx.get();
    if (val) {
      if (Lights.notUsed) {
        Lights.getAllianceIntakeLights();
        lightsTriggered = false;
      }
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
    //if (hasGamePiece()) {
      //stopIntake();
    //}
  }
}
