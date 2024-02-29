// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lift;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lift extends SubsystemBase {
  private TalonFX leftClimber, rightClimber;
  /** Creates a new Lift. */
  public Lift() {
    leftClimber = new TalonFX(57);

    /* Factory Default all hardware to prevent unexpected behaviour */
    leftClimber.configFactoryDefault();

    /* Config the sensor used for Primary PID and sensor direction */
    leftClimber.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor,
        LiftConstants.kPIDLoopIdx,
        LiftConstants.kTimeoutMs);

    /* Ensure sensor is positive when output is positive */
    leftClimber.setSensorPhase(LiftConstants.kSensorPhase);

    /**
     * Set based on what direction you want forward/positive to be. This does not affect sensor
     * phase.
     */
    leftClimber.setInverted(LiftConstants.kMotorInvert);
    /*
     * Talon FX does not need sensor phase set for its integrated sensor
     * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
     * and the user calls getSelectedSensor* to get the sensor's getGripperWristPosition/velocity.
     *
     * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
     */
    // _talon.setSensorPhase(true);

    /* Config the peak and nominal outputs, 12V means full */
    leftClimber.configNominalOutputForward(0, LiftConstants.kTimeoutMs);
    leftClimber.configNominalOutputReverse(0, LiftConstants.kTimeoutMs);
    leftClimber.configPeakOutputForward(1, LiftConstants.kTimeoutMs);
    leftClimber.configPeakOutputReverse(-1, LiftConstants.kTimeoutMs);

    leftClimber.setNeutralMode(NeutralMode.Coast);
    // leftClimber.setNeutralMode(NeutralMode.Brake);

    /**
     * Config the allowable closed-loop error, Closed-Loop output will be neutral within this range.
     * See Table in Section 17.2.1 for native units per rotation.
     */
    leftClimber.configAllowableClosedloopError(
        0, LiftConstants.kPIDLoopIdx, LiftConstants.kTimeoutMs);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */

    leftClimber.config_kF(
        LiftConstants.kPIDLoopIdx, LiftConstants.kGains.kF, LiftConstants.kTimeoutMs);
    leftClimber.config_kP(
        LiftConstants.kPIDLoopIdx, LiftConstants.kGains.kP, LiftConstants.kTimeoutMs);
    leftClimber.config_kI(
        LiftConstants.kPIDLoopIdx, LiftConstants.kGains.kI, LiftConstants.kTimeoutMs);
    leftClimber.config_kD(
        LiftConstants.kPIDLoopIdx, LiftConstants.kGains.kD, LiftConstants.kTimeoutMs);
    leftClimber.config_IntegralZone(
        LiftConstants.kPIDLoopIdx, LiftConstants.kGains.kIzone, LiftConstants.kTimeoutMs);
    leftClimber.configClosedLoopPeakOutput(
        LiftConstants.kPIDLoopIdx, LiftConstants.kGains.kPeakOutput, LiftConstants.kTimeoutMs);

    // leftClimber.config_IntegralZone(LiftConstants.kPIDLoopIdx,
    // LiftConstants.kGains.kIzone, LiftConstants.kTimeoutMs);
    leftClimber.configMotionCruiseVelocity(5500.0);
    leftClimber.configMotionAcceleration(1800.6);
    leftClimber.configMotionSCurveStrength(0);
    leftClimber.setSelectedSensorPosition(0.0, LiftConstants.kPIDLoopIdx, LiftConstants.kTimeoutMs);

    rightClimber = new TalonFX(58);

    /* Factory Default all hardware to prevent unexpected behaviour */
    rightClimber.configFactoryDefault();

    /* Config the sensor used for Primary PID and sensor direction */
    rightClimber.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor,
        LiftConstants.kPIDLoopIdx,
        LiftConstants.kTimeoutMs);

    /* Ensure sensor is positive when output is positive */
    rightClimber.setSensorPhase(LiftConstants.kSensorPhase);

    /**
     * Set based on what direction you want forward/positive to be. This does not affect sensor
     * phase.
     */
    rightClimber.setInverted(LiftConstants.kMotorInvert);
    /*
     * Talon FX does not need sensor phase set for its integrated sensor
     * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
     * and the user calls getSelectedSensor* to get the sensor's getGripperWristPosition/velocity.
     *
     * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
     */
    // _talon.setSensorPhase(true);

    /* Config the peak and nominal outputs, 12V means full */
    rightClimber.configNominalOutputForward(0, LiftConstants.kTimeoutMs);
    rightClimber.configNominalOutputReverse(0, LiftConstants.kTimeoutMs);
    rightClimber.configPeakOutputForward(1, LiftConstants.kTimeoutMs);
    rightClimber.configPeakOutputReverse(-1, LiftConstants.kTimeoutMs);

    rightClimber.setNeutralMode(NeutralMode.Coast);
    // rightClimber.setNeutralMode(NeutralMode.Brake);

    /**
     * Config the allowable closed-loop error, Closed-Loop output will be neutral within this range.
     * See Table in Section 17.2.1 for native units per rotation.
     */
    rightClimber.configAllowableClosedloopError(
        0, LiftConstants.kPIDLoopIdx, LiftConstants.kTimeoutMs);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */

    rightClimber.config_kF(
        LiftConstants.kPIDLoopIdx, LiftConstants.kGains.kF, LiftConstants.kTimeoutMs);
    rightClimber.config_kP(
        LiftConstants.kPIDLoopIdx, LiftConstants.kGains.kP, LiftConstants.kTimeoutMs);
    rightClimber.config_kI(
        LiftConstants.kPIDLoopIdx, LiftConstants.kGains.kI, LiftConstants.kTimeoutMs);
    rightClimber.config_kD(
        LiftConstants.kPIDLoopIdx, LiftConstants.kGains.kD, LiftConstants.kTimeoutMs);
    rightClimber.config_IntegralZone(
        LiftConstants.kPIDLoopIdx, LiftConstants.kGains.kIzone, LiftConstants.kTimeoutMs);
    rightClimber.configClosedLoopPeakOutput(
        LiftConstants.kPIDLoopIdx, LiftConstants.kGains.kPeakOutput, LiftConstants.kTimeoutMs);

    // rightClimber.config_IntegralZone(LiftConstants.kPIDLoopIdx,
    // LiftConstants.kGains.kIzone, LiftConstants.kTimeoutMs);
    rightClimber.configMotionCruiseVelocity(5500.0);
    rightClimber.configMotionAcceleration(1800.6);
    rightClimber.configMotionSCurveStrength(0);
    rightClimber.setSelectedSensorPosition(
        0.0, LiftConstants.kPIDLoopIdx, LiftConstants.kTimeoutMs);
  }

  public void runClimbers(double speed) {
    leftClimber.set(ControlMode.PercentOutput, speed);
    rightClimber.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
