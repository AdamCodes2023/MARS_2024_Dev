// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter_angle;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterAngle extends SubsystemBase {
  private CANSparkMax angleMotor;
  //private SparkAbsoluteEncoder absEncoder;
  //private SparkRelativeEncoder relEncoder;
  private RelativeEncoder relEncoder;
  private SparkPIDController pidControl;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  
  /** Creates a new ShooterAngle. */
  public ShooterAngle() {
    angleMotor = new CANSparkMax(32, MotorType.kBrushless);
    angleMotor.restoreFactoryDefaults();
    angleMotor.setIdleMode(IdleMode.kBrake);
    //absEncoder = angleMotor.getAbsoluteEncoder(Type.kDutyCycle);
    relEncoder = angleMotor.getEncoder();
    pidControl = angleMotor.getPIDController();

    // PID coefficients
    kP = 5e-5; 
    kI = 1e-6;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000156; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    // Smart Motion Coefficients
    maxVel = 5700; // rpm
    maxAcc = 2500;

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
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */
    int smartMotionSlot = 0;
    pidControl.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    pidControl.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    pidControl.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    pidControl.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    createShuffleboard();
  }

  public void createShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("SHOOTER_ANGLE");
    tab.add("SHOOTER_ANGLE", this);
    tab.addNumber("SHOOTER_ANGLE_POSITION", this::getShooterAnglePosition);
  }

  public double getShooterAnglePosition() {
    //return absEncoder.getPosition();
    return relEncoder.getPosition();
  }

  public void smartMotionPosition(double pos) {
    pidControl.setReference(pos, CANSparkMax.ControlType.kSmartMotion);
  }

  public void setMotorSpeed(double speed) {
    angleMotor.set(speed);
  }

  public void stopMotor() {
    angleMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
