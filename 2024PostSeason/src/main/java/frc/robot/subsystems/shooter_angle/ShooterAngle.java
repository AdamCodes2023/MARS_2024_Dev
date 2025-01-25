// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter_angle;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterAngle extends SubsystemBase {
  private boolean looking;
  private DigitalInput hardStopTop;
  private DigitalInput hardStopBottom;
  private CANSparkMax angleMotor;
  private SparkAbsoluteEncoder absEncoder;
  // private SparkRelativeEncoder relEncoder;
  // private SparkMaxAlternateEncoder altEncoder;
  // private RelativeEncoder relEncoder;
  private SparkPIDController pidControl;
  private double angleCorrector;
  private double kP,
      kI,
      kD,
      kIz,
      kFF,
      kMaxOutput,
      kMinOutput,
      maxRPM,
      maxVel,
      minVel,
      maxAcc,
      allowedErr;

  /** Creates a new ShooterAngle. */
  public ShooterAngle() {
    angleCorrector = 0.0;
    looking = true;

    angleMotor = new CANSparkMax(50, MotorType.kBrushless);
    angleMotor.restoreFactoryDefaults();
    angleMotor.setIdleMode(IdleMode.kCoast);
    absEncoder = angleMotor.getAbsoluteEncoder(Type.kDutyCycle);
    absEncoder.setInverted(true);
    absEncoder.setZeroOffset(0.5);
    // absEncoder.setPositionConversionFactor(360.0);
    // relEncoder = angleMotor.getEncoder();

    pidControl = angleMotor.getPIDController();
    pidControl.setFeedbackDevice(absEncoder);
    pidControl.setPositionPIDWrappingEnabled(true);
    pidControl.setPositionPIDWrappingMaxInput(1.0);
    pidControl.setPositionPIDWrappingMinInput(0.0);

    // PID coefficients
    kP = 0.0027;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 1200;
    minVel = 0;

    // Smart Motion Coefficients
    maxVel = 1200; // rpm
    maxAcc = 600;

    // set PID coefficients
    pidControl.setP(kP);
    pidControl.setI(kI);
    pidControl.setD(kD);
    pidControl.setIZone(kIz);
    pidControl.setFF(kFF);
    pidControl.setOutputRange(kMinOutput, kMaxOutput);

    /**
     * Smart Motion coefficients are set on a SparkPIDController object
     *
     * <p>- setSmartMotionMaxVelocity() will limit the velocity in RPM of the pid controller in
     * Smart Motion mode - setSmartMotionMinOutputVelocity() will put a lower bound in RPM of the
     * pid controller in Smart Motion mode - setSmartMotionMaxAccel() will limit the acceleration in
     * RPM^2 of the pid controller in Smart Motion mode - setSmartMotionAllowedClosedLoopError()
     * will set the max allowed error for the pid controller in Smart Motion mode
     */
    int smartMotionSlot = 0;
    pidControl.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    pidControl.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    pidControl.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    pidControl.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    hardStopBottom = new DigitalInput(2);
    hardStopTop = new DigitalInput(3);

    createShuffleboard();
  }

  public void createShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("SHOOTER_ANGLE");
    tab.add("SHOOTER_ANGLE", this);
    tab.addNumber("SHOOTER_ANGLE_POSITION", this::getShooterAnglePosition);
    tab.addBoolean("HardStopTop", this::atHardStopTop);
    tab.addBoolean("HardStopBottom", this::atHardStopBottom);
    tab.addNumber("Output Current", this::getCurrent);
    tab.addNumber("Angle Corrector", this::getAngleCorrector);
  }

  public double getCurrent() {
    return angleMotor.getOutputCurrent();
  }

  public double getAngleCorrector() {
    return angleCorrector;
  }

  public void increaseAngleCorrector() {
    angleCorrector += 0.5;
  }

  public void decreaseAngleCorrector() {
    angleCorrector -= 0.5;
  }

  public double getShooterAnglePosition() {
    return absEncoder.getPosition() * 360.0;
    // return relEncoder.getPosition();
  }

  public void smartMotionPosition(double pos) {
    pidControl.setReference(pos + (angleCorrector/360.0), CANSparkMax.ControlType.kSmartMotion);
    // pidControl.setReference(pos, CANSparkMax.ControlType.kPosition);
  }

  public void setMotorSpeed(double speed) {
    angleMotor.set(speed);
  }

  public void stopMotor() {
    angleMotor.stopMotor();
  }

  public void stopMotorWithInterrupt(boolean interrupted) {
    angleMotor.stopMotor();
  }

  public void intakePosition() {
    smartMotionPosition(179.6 / 360.0);
  }

  public void ampPosition() {
    smartMotionPosition(263.0 / 360.0);
  }

  public void speakerPosition() {
    smartMotionPosition(179.6 / 360.0);
  }

  public void setPosition(double pos) {
    smartMotionPosition(pos / 360.0);
  }

  public boolean atIntakeAlignment() {
    return getShooterAnglePosition() > (178.6 + angleCorrector)
        && getShooterAnglePosition() < (180.6 + angleCorrector);
  }

  public boolean atAmpAlignment() {
    return getShooterAnglePosition() > (262.0 + angleCorrector)
        && getShooterAnglePosition() < (264.0 + angleCorrector);
  }

  public boolean atSpeakerAlignment() {
    return getShooterAnglePosition() > (178.6 + angleCorrector)
        && getShooterAnglePosition() < (180.6 + angleCorrector);
  }

  public boolean atSetAlignment(double pos) {
    return getShooterAnglePosition() > (pos - 1.0 + angleCorrector)
        && getShooterAnglePosition() < (pos + 1.0 + angleCorrector);
  }

  public boolean atHardStopTop() {
    return !hardStopTop.get();
  }

  public boolean atHardStopBottom() {
    return !hardStopBottom.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (atHardStopBottom() || atHardStopTop()) {
      if (looking) {
        stopMotor();
        looking = false;
      }
    } else {
      looking = true;
    }
  }
}
