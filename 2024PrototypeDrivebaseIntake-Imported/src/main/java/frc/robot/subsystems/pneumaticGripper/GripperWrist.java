// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pneumaticGripper;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.Elevator;

public class GripperWrist extends SubsystemBase {
  /** Creates a new GripperWrist. */
  TalonFX gripperWristMotor;

  private static int addShuffleboard = 0;

  public GripperWrist() {
    gripperWristMotor = new TalonFX(GripperWristConstants.gripperWristMotorID);

    /* Factory Default all hardware to prevent unexpected behaviour */
    gripperWristMotor.configFactoryDefault();

    gripperWristMotor.configForwardLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    gripperWristMotor.configReverseLimitSwitchSource(
        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);

    /* Config the sensor used for Primary PID and sensor direction */
    gripperWristMotor.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor,
        GripperWristConstants.kPIDLoopIdx,
        GripperWristConstants.kTimeoutMs);

    /* Ensure sensor is positive when output is positive */
    gripperWristMotor.setSensorPhase(GripperWristConstants.kSensorPhase);

    /**
     * Set based on what direction you want forward/positive to be. This does not affect sensor
     * phase.
     */
    gripperWristMotor.setInverted(GripperWristConstants.kMotorInvert);
    /*
     * Talon FX does not need sensor phase set for its integrated sensor
     * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
     * and the user calls getSelectedSensor* to get the sensor's getGripperWristPosition/velocity.
     *
     * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
     */
    // _talon.setSensorPhase(true);

    /* Config the peak and nominal outputs, 12V means full */
    gripperWristMotor.configNominalOutputForward(0, GripperWristConstants.kTimeoutMs);
    gripperWristMotor.configNominalOutputReverse(0, GripperWristConstants.kTimeoutMs);
    gripperWristMotor.configPeakOutputForward(1, GripperWristConstants.kTimeoutMs);
    gripperWristMotor.configPeakOutputReverse(-1, GripperWristConstants.kTimeoutMs);

    // gripperWristMotor.setNeutralMode(NeutralMode.Coast);
    gripperWristMotor.setNeutralMode(NeutralMode.Brake);

    /**
     * Config the allowable closed-loop error, Closed-Loop output will be neutral within this range.
     * See Table in Section 17.2.1 for native units per rotation.
     */
    gripperWristMotor.configAllowableClosedloopError(
        0, GripperWristConstants.kPIDLoopIdx, GripperWristConstants.kTimeoutMs);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
    gripperWristMotor.config_kF(
        GripperWristConstants.kPIDLoopIdx,
        GripperWristConstants.kGains.kF,
        GripperWristConstants.kTimeoutMs);
    gripperWristMotor.config_kP(
        GripperWristConstants.kPIDLoopIdx,
        GripperWristConstants.kGains.kP,
        GripperWristConstants.kTimeoutMs);
    gripperWristMotor.config_kI(
        GripperWristConstants.kPIDLoopIdx,
        GripperWristConstants.kGains.kI,
        GripperWristConstants.kTimeoutMs);
    gripperWristMotor.config_kD(
        GripperWristConstants.kPIDLoopIdx,
        GripperWristConstants.kGains.kD,
        GripperWristConstants.kTimeoutMs);
    gripperWristMotor.config_IntegralZone(
        GripperWristConstants.kPIDLoopIdx,
        GripperWristConstants.kGains.kIzone,
        GripperWristConstants.kTimeoutMs);
    gripperWristMotor.configClosedLoopPeakOutput(
        GripperWristConstants.kPIDLoopIdx,
        GripperWristConstants.kGains.kPeakOutput,
        GripperWristConstants.kTimeoutMs);

    // gripperWristMotor.config_IntegralZone(GripperWristConstants.kPIDLoopIdx,
    // GripperWristConstants.kGains.kIzone, GripperWristConstants.kTimeoutMs);
    gripperWristMotor.configMotionCruiseVelocity(2797.8);
    gripperWristMotor.configMotionAcceleration(932.6);
    gripperWristMotor.configMotionSCurveStrength(0);
    gripperWristMotor.setSelectedSensorPosition(
        0.0, GripperWristConstants.kPIDLoopIdx, GripperWristConstants.kTimeoutMs);

    createShuffleboard();
  }

  public void createShuffleboard() {
    if (addShuffleboard < 1) {
      ShuffleboardTab tab = Shuffleboard.getTab("GRIPPERTAB");
      tab.add("GRIPPER", this);
      tab.addNumber("GRIPPER POSITION", this::getGripperWristPosition);
      tab.addNumber("GRIPPER CURRENT", this::getGripperWristCurrent);
      tab.addNumber("GRIPPER TEMP", this::getGripperWristTemp);
      addShuffleboard++;
    }
  }

  public void stop() {
    gripperWristMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void resetSensorPosition() {
    gripperWristMotor.setSelectedSensorPosition(
        0.0, GripperWristConstants.kPIDLoopIdx, GripperWristConstants.kTimeoutMs);
  }

  public int returnForwardLimitSwitchState() {
    // return gripperWristMotor.getSensorCollection().isFwdLimitSwitchClosed();
    return gripperWristMotor.isFwdLimitSwitchClosed();
  }

  public int returnReverseLimitSwitchState() {
    return gripperWristMotor.isRevLimitSwitchClosed();
  }

  public void run(double speed) {
    gripperWristMotor.set(ControlMode.PercentOutput, speed);
  }

  public void runUntilForwardLimit() {
    if (returnForwardLimitSwitchState() == 0) {
      run(0.1);
    } else {
      stop();
      // resetSensorPosition();
    }
    /***** HAVE NOT TESTED LIMIT SWITCHES AND POSITION MOVEMENT YET *****/
    // gripperWristMotor.setInverted(true);
  }

  public void runUntilReverseLimit() {
    if (returnReverseLimitSwitchState() == 0 /*&& Elevator.getReverseLimitValue() == 1*/) {
      run(-0.1);
    } else {
      stop();
      resetSensorPosition();
    }
    /***** HAVE NOT TESTED LIMIT SWITCHES AND POSITION MOVEMENT YET *****/
    // gripperWristMotor.setInverted(false);
  }

  public void runToPosition(double pos) {
    /***** NO ACTIVE ENCODER YET *****/
    gripperWristMotor.set(
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

  public void runToElevatorOppositePos() {
    gripperWristMotor.set(
        ControlMode.MotionMagic,
        (90.0 - (Elevator.getStaticAngle())) * (36182.0 / 90.0),
        DemandType.ArbitraryFeedForward,
        getFeedForward());
  }

  public void runToElevatorSamePos() {
    gripperWristMotor.set(
        ControlMode.MotionMagic,
        (Elevator.getStaticAngle()) * (36182.0 / 90.0),
        DemandType.ArbitraryFeedForward,
        getFeedForward());
  }

  public double getGripperWristPosition() {
    return gripperWristMotor.getSelectedSensorPosition();
  }

  public double getGripperWristCurrent() {
    return gripperWristMotor.getStatorCurrent();
  }

  public double getGripperWristTemp() {
    return gripperWristMotor.getTemperature();
  }

  // return the angle of the arm based on the current encoder value
  public double getAngle() {
    double currentPosition = getGripperWristPosition();

    // divide the encoder position when arm is horizontal by the angle displacement
    // so if you moved the arm 30 degrees and read 1000 ticks, it would be 1000/30 ticks per degree
    // -10000
    // subtract horizontalAngleDisplacement to make the horizontal postion 0 degrees
    double ticksPerDegree = 36182.0 / 90.0;
    double angle = currentPosition / ticksPerDegree - 0;
    return angle;
  }

  public double getFeedForward() {

    // get the radians of the arm
    // getAngle() returns degrees
    double theta = Math.toRadians(getAngle());

    SmartDashboard.putNumber("Gripper Angle", getAngle());

    // get a range of 0 to 1 to multiply by feedforward.
    // when in horizontal position, value should be 1
    // when in vertical up or down position, value should be 0
    double gravityCompensation = Math.cos(theta);

    SmartDashboard.putNumber("Gripper Gravity Compensation", gravityCompensation);

    // horizontalHoldOutput is the minimum power required to hold the arm up when horizontal
    // this is a range of 0-1, in our case it was .125 throttle required to keep the arm up
    double feedForward = gravityCompensation * 0.2; // <- horizontalHoldOutput

    SmartDashboard.putNumber("Gripper Feed Forward", feedForward);

    return feedForward;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
