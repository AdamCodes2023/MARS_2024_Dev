// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  TalonFX elevatorMotor;
  TalonFX elevatorMotorSlave;
  public static int forwardLimitValue;
  public static int reverseLimitValue;
  public static double staticAngle;
  private static int addShuffleboard = 0;
  /** Creates a new Elevator. */
  public Elevator() {

    elevatorMotor = new TalonFX(ElevatorConstants.elevatorMotorID);

    /* Factory Default all hardware to prevent unexpected behaviour */
    elevatorMotor.configFactoryDefault();

    elevatorMotor.configForwardLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    elevatorMotor.configReverseLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);

    /* Config the sensor used for Primary PID and sensor direction */
    elevatorMotor.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor,
        ElevatorConstants.kPIDLoopIdx,
        ElevatorConstants.kTimeoutMs);

    /* Ensure sensor is positive when output is positive */
    elevatorMotor.setSensorPhase(ElevatorConstants.kSensorPhase);

    /**
     * Set based on what direction you want forward/positive to be. This does not affect sensor
     * phase.
     */
    elevatorMotor.setInverted(ElevatorConstants.kMotorInvert);
    /*
     * Talon FX does not need sensor phase set for its integrated sensor
     * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
     * and the user calls getSelectedSensor* to get the sensor's getGripperWristPosition/velocity.
     *
     * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
     */
    // _talon.setSensorPhase(true);

    /* Config the peak and nominal outputs, 12V means full */
    elevatorMotor.configNominalOutputForward(0, ElevatorConstants.kTimeoutMs);
    elevatorMotor.configNominalOutputReverse(0, ElevatorConstants.kTimeoutMs);
    elevatorMotor.configPeakOutputForward(1, ElevatorConstants.kTimeoutMs);
    elevatorMotor.configPeakOutputReverse(-1, ElevatorConstants.kTimeoutMs);

    elevatorMotor.setNeutralMode(NeutralMode.Coast);
    // elevatorMotor.setNeutralMode(NeutralMode.Brake);

    /**
     * Config the allowable closed-loop error, Closed-Loop output will be neutral within this range.
     * See Table in Section 17.2.1 for native units per rotation.
     */
    elevatorMotor.configAllowableClosedloopError(
        0, ElevatorConstants.kPIDLoopIdx, ElevatorConstants.kTimeoutMs);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */

    elevatorMotor.config_kF(
        ElevatorConstants.kPIDLoopIdx, ElevatorConstants.kGains.kF, ElevatorConstants.kTimeoutMs);
    elevatorMotor.config_kP(
        ElevatorConstants.kPIDLoopIdx, ElevatorConstants.kGains.kP, ElevatorConstants.kTimeoutMs);
    elevatorMotor.config_kI(
        ElevatorConstants.kPIDLoopIdx, ElevatorConstants.kGains.kI, ElevatorConstants.kTimeoutMs);
    elevatorMotor.config_kD(
        ElevatorConstants.kPIDLoopIdx, ElevatorConstants.kGains.kD, ElevatorConstants.kTimeoutMs);
    elevatorMotor.config_IntegralZone(
        ElevatorConstants.kPIDLoopIdx,
        ElevatorConstants.kGains.kIzone,
        ElevatorConstants.kTimeoutMs);
    elevatorMotor.configClosedLoopPeakOutput(
        ElevatorConstants.kPIDLoopIdx,
        ElevatorConstants.kGains.kPeakOutput,
        ElevatorConstants.kTimeoutMs);

    // elevatorMotor.config_IntegralZone(ElevatorConstants.kPIDLoopIdx,
    // ElevatorConstants.kGains.kIzone, ElevatorConstants.kTimeoutMs);
    elevatorMotor.configMotionCruiseVelocity(5500.0);
    elevatorMotor.configMotionAcceleration(1800.6);
    elevatorMotor.configMotionSCurveStrength(0);
    elevatorMotor.setSelectedSensorPosition(
        0.0, ElevatorConstants.kPIDLoopIdx, ElevatorConstants.kTimeoutMs);

    elevatorMotorSlave = new TalonFX(ElevatorConstants.elevatorMotorSlaveID);

    /* Factory Default all hardware to prevent unexpected behaviour */
    elevatorMotorSlave.configFactoryDefault();

    elevatorMotorSlave.configForwardLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    elevatorMotorSlave.configReverseLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);

    /* Config the sensor used for Primary PID and sensor direction */

    elevatorMotorSlave.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor,
        ElevatorConstants.kPIDLoopIdx,
        ElevatorConstants.kTimeoutMs);

    /* Ensure sensor is positive when output is positive */
    elevatorMotorSlave.setSensorPhase(ElevatorConstants.kSensorPhase);

    /**
     * Set based on what direction you want forward/positive to be. This does not affect sensor
     * phase.
     */
    elevatorMotorSlave.setInverted(ElevatorConstants.kMotorInvert);
    /*
     * Talon FX does not need sensor phase set for its integrated sensor
     * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
     * and the user calls getSelectedSensor* to get the sensor's getGripperWristPosition/velocity.
     *
     * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
     */
    // _talon.setSensorPhase(true);

    /* Config the peak and nominal outputs, 12V means full */

    elevatorMotorSlave.configNominalOutputForward(0, ElevatorConstants.kTimeoutMs);
    elevatorMotorSlave.configNominalOutputReverse(0, ElevatorConstants.kTimeoutMs);
    elevatorMotorSlave.configPeakOutputForward(1, ElevatorConstants.kTimeoutMs);
    elevatorMotorSlave.configPeakOutputReverse(-1, ElevatorConstants.kTimeoutMs);

    elevatorMotorSlave.setNeutralMode(NeutralMode.Coast);

    // elevatorMotorSlave.setNeutralMode(NeutralMode.Brake);

    /**
     * Config the allowable closed-loop error, Closed-Loop output will be neutral within this range.
     * See Table in Section 17.2.1 for native units per rotation.
     */
    elevatorMotorSlave.configAllowableClosedloopError(
        0, ElevatorConstants.kPIDLoopIdx, ElevatorConstants.kTimeoutMs);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */

    elevatorMotorSlave.config_kF(
        ElevatorConstants.kPIDLoopIdx, ElevatorConstants.kGains.kF, ElevatorConstants.kTimeoutMs);
    elevatorMotorSlave.config_kP(
        ElevatorConstants.kPIDLoopIdx, ElevatorConstants.kGains.kP, ElevatorConstants.kTimeoutMs);
    elevatorMotorSlave.config_kI(
        ElevatorConstants.kPIDLoopIdx, ElevatorConstants.kGains.kI, ElevatorConstants.kTimeoutMs);
    elevatorMotorSlave.config_kD(
        ElevatorConstants.kPIDLoopIdx, ElevatorConstants.kGains.kD, ElevatorConstants.kTimeoutMs);
    elevatorMotorSlave.config_IntegralZone(
        ElevatorConstants.kPIDLoopIdx,
        ElevatorConstants.kGains.kIzone,
        ElevatorConstants.kTimeoutMs);
    elevatorMotorSlave.configClosedLoopPeakOutput(
        ElevatorConstants.kPIDLoopIdx,
        ElevatorConstants.kGains.kPeakOutput,
        ElevatorConstants.kTimeoutMs);

    // elevatorMotorSlave.config_IntegralZone(ElevatorConstants.kPIDLoopIdx,
    // ElevatorConstants.kGains.kIzone, ElevatorConstants.kTimeoutMs);

    elevatorMotorSlave.configMotionCruiseVelocity(5500.0);
    elevatorMotorSlave.configMotionAcceleration(1800.6);
    elevatorMotorSlave.configMotionSCurveStrength(0);
    elevatorMotorSlave.setSelectedSensorPosition(
        0.0, ElevatorConstants.kPIDLoopIdx, ElevatorConstants.kTimeoutMs);

    elevatorMotorSlave.follow(elevatorMotor);
    elevatorMotorSlave.setInverted(InvertType.FollowMaster);

    createShuffleboard();
  }

  public void createShuffleboard() {
    if (addShuffleboard < 1) {
      ShuffleboardTab tab = Shuffleboard.getTab("ELEVATORTAB");
      tab.add("ELEVATOR", this);
      tab.addNumber("ELEVATOR POSITION", this::getElevatorPosition);
      tab.addNumber("ELEVATOR CURRENT", this::getElevatorCurrent);
      tab.addNumber("ELEVATOR TEMP", this::getElevatorTemp);
      addShuffleboard++;
    }
  }

  public void stop() {
    elevatorMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void resetSensorPosition() {
    elevatorMotor.setSelectedSensorPosition(
        0.0, ElevatorConstants.kPIDLoopIdx, ElevatorConstants.kTimeoutMs);
  }

  public int returnForwardLimitSwitchState() {
    // return elevatorMotor.getSensorCollection().isFwdLimitSwitchClosed();
    setForwardLimitValue(elevatorMotor.isFwdLimitSwitchClosed());
    return elevatorMotor.isFwdLimitSwitchClosed();
  }

  public int returnReverseLimitSwitchState() {
    setReverseLimitValue(elevatorMotor.isRevLimitSwitchClosed());
    return elevatorMotor.isRevLimitSwitchClosed();
  }

  private void setForwardLimitValue(int limit) {
    forwardLimitValue = limit;
  }

  public static int getForwardLimitValue() {
    return forwardLimitValue;
  }

  private void setReverseLimitValue(int limit) {
    reverseLimitValue = limit;
  }

  public static int getReverseLimitValue() {
    return reverseLimitValue;
  }

  public void run(double speed) {
    elevatorMotor.set(ControlMode.PercentOutput, speed);
  }

  public void runUntilForwardLimit() {
    if (returnForwardLimitSwitchState() == 0) {
      run(0.7);
    } else {
      stop();
      // resetSensorPosition();
    }
    /***** HAVE NOT TESTED LIMIT SWITCHES AND POSITION MOVEMENT YET *****/
    // elevatorMotor.setInverted(true);
  }

  public void runUntilReverseLimit() {
    if (returnReverseLimitSwitchState() == 0) {
      run(-0.7);
    } else {
      stop();
      resetSensorPosition();
    }
    /***** HAVE NOT TESTED LIMIT SWITCHES AND POSITION MOVEMENT YET *****/
    // elevatorMotor.setInverted(false);
  }

  public void runToPosition(double pos) {
    /***** NO ACTIVE ENCODER YET *****/
    elevatorMotor.set(
        ControlMode.MotionMagic, pos, DemandType.ArbitraryFeedForward, getFeedForward());

    /*
    if (pos > getGripperWristPosition()) {
      if (getGripperWristPosition() >= pos - 10 && getGripperWristPosition() <= pos + 10) {
        stop();
      }
      else {
        run(0.1);
      }
    }
    else {
      if (getGripperWristPosition() >= pos - 10 && getGripperWristPosition() <= pos + 10) {
        stop();
      }
      else {
        run(-0.1);
      }
    }
    */
  }

  // FIXME: SET CORRECT POSITIONS FOR ELEVATOR MOTORS
  public void runToDownPosition() {
    runToPosition(0);
  }

  public void runToGroundScorePosition() {
    runToPosition(9);
  }

  public void runToHumanPickUpPosition() {
    runToPosition(0);
  }

  public void runToTravelPosition() {
    runToPosition(0);
    // runUntilReverseLimit();
  }

  public void runToLowScorePosition() {
    runToPosition(174800);
  }

  public void runToHighScorePosition() {
    runToPosition(207300);
  }

  public double getElevatorPosition() {
    return elevatorMotor.getSelectedSensorPosition();
  }

  public double getElevatorCurrent() {
    return elevatorMotor.getStatorCurrent();
  }

  public double getElevatorTemp() {
    return elevatorMotor.getTemperature();
  }

  // return the angle of the arm based on the current encoder value
  public double getAngle() {
    double currentPosition = getElevatorPosition();

    // divide the encoder position when arm is horizontal by the angle displacement
    // so if you moved the arm 30 degrees and read 1000 ticks, it would be 1000/30 ticks per degree
    // subtract horizontalAngleDisplacement to make the horizontal postion 0 degrees
    // FIXME: Set ticksPerDegree for Elevator.
    double ticksPerDegree = 207300.0 / 40.0;
    double angle = currentPosition / ticksPerDegree - 0;
    setAngle(angle);
    return angle;
  }

  private void setAngle(double a) {
    staticAngle = a;
  }

  public static double getStaticAngle() {
    return staticAngle;
  }

  public double getFeedForward() {

    // get the radians of the arm
    // getAngle() returns degrees
    double theta = Math.toRadians(getAngle());

    SmartDashboard.putNumber(" Elevator Angle", getAngle());

    // get a range of 0 to 1 to multiply by feedforward.
    // when in horizontal position, value should be 1
    // when in vertical up or down position, value should be 0
    double gravityCompensation = Math.cos(theta);

    SmartDashboard.putNumber("Elevator Gravity Compensation", gravityCompensation);

    // horizontalHoldOutput is the minimum power required to hold the arm up when horizontal
    // this is a range of 0-1, in our case it was .125 throttle required to keep the arm up
    double feedForward = gravityCompensation * 0.2; // <- horizontalHoldOutput

    SmartDashboard.putNumber("Elevator Feed Forward", feedForward);

    return feedForward;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if (ElevatorCarriage.getPos() > -10000.0 && ElevatorCarriage.getPos() < -1000) {
    // stop();
    // }
  }
}
