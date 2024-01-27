// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private TalonFX leftShooter, rightShooter;
  /** Creates a new Shooter. */
  public Shooter() {
    leftShooter = new TalonFX(55);

    /* Factory Default all hardware to prevent unexpected behaviour */
    leftShooter.configFactoryDefault();

    /* Config the sensor used for Primary PID and sensor direction */
    leftShooter.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor,
        ShooterConstants.kPIDLoopIdx,
        ShooterConstants.kTimeoutMs);

    /* Ensure sensor is positive when output is positive */
    leftShooter.setSensorPhase(ShooterConstants.kSensorPhase);

    /**
     * Set based on what direction you want forward/positive to be. This does not affect sensor
     * phase.
     */
    leftShooter.setInverted(ShooterConstants.kMotorInvert);
    /*
     * Talon FX does not need sensor phase set for its integrated sensor
     * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
     * and the user calls getSelectedSensor* to get the sensor's getGripperWristPosition/velocity.
     *
     * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
     */
    // _talon.setSensorPhase(true);

    /* Config the peak and nominal outputs, 12V means full */
    leftShooter.configNominalOutputForward(0, ShooterConstants.kTimeoutMs);
    leftShooter.configNominalOutputReverse(0, ShooterConstants.kTimeoutMs);
    leftShooter.configPeakOutputForward(1, ShooterConstants.kTimeoutMs);
    leftShooter.configPeakOutputReverse(-1, ShooterConstants.kTimeoutMs);

    leftShooter.setNeutralMode(NeutralMode.Coast);
    // leftShooter.setNeutralMode(NeutralMode.Brake);

    /**
     * Config the allowable closed-loop error, Closed-Loop output will be neutral within this range.
     * See Table in Section 17.2.1 for native units per rotation.
     */
    leftShooter.configAllowableClosedloopError(
        0, ShooterConstants.kPIDLoopIdx, ShooterConstants.kTimeoutMs);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */

    leftShooter.config_kF(
        ShooterConstants.kPIDLoopIdx, ShooterConstants.kGains.kF, ShooterConstants.kTimeoutMs);
    leftShooter.config_kP(
        ShooterConstants.kPIDLoopIdx, ShooterConstants.kGains.kP, ShooterConstants.kTimeoutMs);
    leftShooter.config_kI(
        ShooterConstants.kPIDLoopIdx, ShooterConstants.kGains.kI, ShooterConstants.kTimeoutMs);
    leftShooter.config_kD(
        ShooterConstants.kPIDLoopIdx, ShooterConstants.kGains.kD, ShooterConstants.kTimeoutMs);
    leftShooter.config_IntegralZone(
        ShooterConstants.kPIDLoopIdx,
        ShooterConstants.kGains.kIzone,
        ShooterConstants.kTimeoutMs);
    leftShooter.configClosedLoopPeakOutput(
        ShooterConstants.kPIDLoopIdx,
        ShooterConstants.kGains.kPeakOutput,
        ShooterConstants.kTimeoutMs);

    // leftShooter.config_IntegralZone(ShooterConstants.kPIDLoopIdx,
    // ShooterConstants.kGains.kIzone, ShooterConstants.kTimeoutMs);
    leftShooter.configMotionCruiseVelocity(5500.0);
    leftShooter.configMotionAcceleration(1800.6);
    leftShooter.configMotionSCurveStrength(0);
    leftShooter.setSelectedSensorPosition(
        0.0, ShooterConstants.kPIDLoopIdx, ShooterConstants.kTimeoutMs);

    rightShooter = new TalonFX(56);

    /* Factory Default all hardware to prevent unexpected behaviour */
    rightShooter.configFactoryDefault();

    /* Config the sensor used for Primary PID and sensor direction */
    rightShooter.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor,
        ShooterConstants.kPIDLoopIdx,
        ShooterConstants.kTimeoutMs);

    /* Ensure sensor is positive when output is positive */
    rightShooter.setSensorPhase(ShooterConstants.kSensorPhase);

    /**
     * Set based on what direction you want forward/positive to be. This does not affect sensor
     * phase.
     */
    rightShooter.setInverted(ShooterConstants.kMotorInvert);
    /*
     * Talon FX does not need sensor phase set for its integrated sensor
     * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
     * and the user calls getSelectedSensor* to get the sensor's getGripperWristPosition/velocity.
     *
     * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
     */
    // _talon.setSensorPhase(true);

    /* Config the peak and nominal outputs, 12V means full */
    rightShooter.configNominalOutputForward(0, ShooterConstants.kTimeoutMs);
    rightShooter.configNominalOutputReverse(0, ShooterConstants.kTimeoutMs);
    rightShooter.configPeakOutputForward(1, ShooterConstants.kTimeoutMs);
    rightShooter.configPeakOutputReverse(-1, ShooterConstants.kTimeoutMs);

    rightShooter.setNeutralMode(NeutralMode.Coast);
    // rightShooter.setNeutralMode(NeutralMode.Brake);

    /**
     * Config the allowable closed-loop error, Closed-Loop output will be neutral within this range.
     * See Table in Section 17.2.1 for native units per rotation.
     */
    rightShooter.configAllowableClosedloopError(
        0, ShooterConstants.kPIDLoopIdx, ShooterConstants.kTimeoutMs);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */

    rightShooter.config_kF(
        ShooterConstants.kPIDLoopIdx, ShooterConstants.kGains.kF, ShooterConstants.kTimeoutMs);
    rightShooter.config_kP(
        ShooterConstants.kPIDLoopIdx, ShooterConstants.kGains.kP, ShooterConstants.kTimeoutMs);
    rightShooter.config_kI(
        ShooterConstants.kPIDLoopIdx, ShooterConstants.kGains.kI, ShooterConstants.kTimeoutMs);
    rightShooter.config_kD(
        ShooterConstants.kPIDLoopIdx, ShooterConstants.kGains.kD, ShooterConstants.kTimeoutMs);
    rightShooter.config_IntegralZone(
        ShooterConstants.kPIDLoopIdx,
        ShooterConstants.kGains.kIzone,
        ShooterConstants.kTimeoutMs);
    rightShooter.configClosedLoopPeakOutput(
        ShooterConstants.kPIDLoopIdx,
        ShooterConstants.kGains.kPeakOutput,
        ShooterConstants.kTimeoutMs);

    // rightShooter.config_IntegralZone(ShooterConstants.kPIDLoopIdx,
    // ShooterConstants.kGains.kIzone, ShooterConstants.kTimeoutMs);
    rightShooter.configMotionCruiseVelocity(5500.0);
    rightShooter.configMotionAcceleration(1800.6);
    rightShooter.configMotionSCurveStrength(0);
    rightShooter.setSelectedSensorPosition(
        0.0, ShooterConstants.kPIDLoopIdx, ShooterConstants.kTimeoutMs);
  }

  public void runShooter(double speed) {
    leftShooter.set(ControlMode.Velocity, speed);
    rightShooter.set(ControlMode.Velocity, -speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
