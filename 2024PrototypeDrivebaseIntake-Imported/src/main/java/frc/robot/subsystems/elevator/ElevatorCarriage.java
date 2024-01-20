// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

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

public class ElevatorCarriage extends SubsystemBase {
  /** Creates a new ElevatorCarriage. */
  TalonFX elevatorCarriageMotor;

  public static double pos;
  private static int addShuffleboard = 0;

  public ElevatorCarriage() {
    elevatorCarriageMotor = new TalonFX(ElevatorConstants.elevatorCarriageMotorID);

    createShuffleboard();
  }

  public void createShuffleboard() {
    if (addShuffleboard < 1) {
      /* Factory Default all hardware to prevent unexpected behaviour */
      elevatorCarriageMotor.configFactoryDefault();

      elevatorCarriageMotor.configForwardLimitSwitchSource(
          LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
      elevatorCarriageMotor.configReverseLimitSwitchSource(
          LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);

      /* Config the sensor used for Primary PID and sensor direction */
      elevatorCarriageMotor.configSelectedFeedbackSensor(
          TalonFXFeedbackDevice.IntegratedSensor,
          ElevatorConstants.kPIDLoopIdx,
          ElevatorConstants.kTimeoutMs);

      /* Ensure sensor is positive when output is positive */
      elevatorCarriageMotor.setSensorPhase(ElevatorConstants.kSensorPhase);

      /**
       * Set based on what direction you want forward/positive to be. This does not affect sensor
       * phase.
       */
      elevatorCarriageMotor.setInverted(ElevatorConstants.kMotorInvert);
      /*
       * Talon FX does not need sensor phase set for its integrated sensor
       * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
       * and the user calls getSelectedSensor* to get the sensor's getGripperWristPosition/velocity.
       *
       * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
       */
      // _talon.setSensorPhase(true);

      /* Config the peak and nominal outputs, 12V means full */
      elevatorCarriageMotor.configNominalOutputForward(0, ElevatorConstants.kTimeoutMs);
      elevatorCarriageMotor.configNominalOutputReverse(0, ElevatorConstants.kTimeoutMs);
      elevatorCarriageMotor.configPeakOutputForward(1, ElevatorConstants.kTimeoutMs);
      elevatorCarriageMotor.configPeakOutputReverse(-1, ElevatorConstants.kTimeoutMs);

      // elevatorCarriageMotor.setNeutralMode(NeutralMode.Coast);
      elevatorCarriageMotor.setNeutralMode(NeutralMode.Brake);

      /**
       * Config the allowable closed-loop error, Closed-Loop output will be neutral within this
       * range. See Table in Section 17.2.1 for native units per rotation.
       */
      elevatorCarriageMotor.configAllowableClosedloopError(
          0, ElevatorConstants.kPIDLoopIdx, ElevatorConstants.kTimeoutMs);

      /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
      elevatorCarriageMotor.config_kF(
          ElevatorConstants.kPIDLoopIdx, ElevatorConstants.kGains.kF, ElevatorConstants.kTimeoutMs);
      elevatorCarriageMotor.config_kP(
          ElevatorConstants.kPIDLoopIdx, ElevatorConstants.kGains.kP, ElevatorConstants.kTimeoutMs);
      elevatorCarriageMotor.config_kI(
          ElevatorConstants.kPIDLoopIdx, ElevatorConstants.kGains.kI, ElevatorConstants.kTimeoutMs);
      elevatorCarriageMotor.config_kD(
          ElevatorConstants.kPIDLoopIdx, ElevatorConstants.kGains.kD, ElevatorConstants.kTimeoutMs);
      elevatorCarriageMotor.config_IntegralZone(
          ElevatorConstants.kPIDLoopIdx,
          ElevatorConstants.kGains.kIzone,
          ElevatorConstants.kTimeoutMs);
      elevatorCarriageMotor.configClosedLoopPeakOutput(
          ElevatorConstants.kPIDLoopIdx,
          ElevatorConstants.kGains.kPeakOutput,
          ElevatorConstants.kTimeoutMs);

      // elevatorCarriageMotor.config_IntegralZone(ElevatorConstants.kPIDLoopIdx,
      // ElevatorConstants.kGains.kIzone, ElevatorConstants.kTimeoutMs);
      elevatorCarriageMotor.configMotionCruiseVelocity(2797.8);
      elevatorCarriageMotor.configMotionAcceleration(932.6);
      elevatorCarriageMotor.configMotionSCurveStrength(0);
      elevatorCarriageMotor.setSelectedSensorPosition(
          0.0, ElevatorConstants.kPIDLoopIdx, ElevatorConstants.kTimeoutMs);

      elevatorCarriageMotor.setNeutralMode(NeutralMode.Brake);
      ShuffleboardTab tab = Shuffleboard.getTab("ELEVATORCARRIAGETAB");
      tab.add("ELEVATORCARRIAGE", this);
      tab.addNumber("ELEVATOR CARRIAGE POSITION", this::getElevatorCarriagePosition);
      addShuffleboard++;
    }
  }

  public void stopCarriage() {
    elevatorCarriageMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void resetCarriageSensorPosition() {
    elevatorCarriageMotor.setSelectedSensorPosition(
        0.0, ElevatorConstants.kPIDLoopIdx, ElevatorConstants.kTimeoutMs);
  }

  public int returnCarriageForwardLimitSwitchState() {
    // return elevatorCarriageMotor.getSensorCollection().isFwdLimitSwitchClosed();
    return elevatorCarriageMotor.isFwdLimitSwitchClosed();
  }

  public int returnCarriageReverseLimitSwitchState() {
    return elevatorCarriageMotor.isRevLimitSwitchClosed();
  }

  public void runCarriage(double speed) {
    elevatorCarriageMotor.set(ControlMode.PercentOutput, speed);
  }

  public void runCarriageUntilForwardLimit() {
    if (returnCarriageForwardLimitSwitchState() == 0 && Elevator.getReverseLimitValue() == 1) {
      runCarriage(0.1);
    } else {
      stopCarriage();
      // resetCarriageSensorPosition();
    }
    /***** HAVE NOT TESTED LIMIT SWITCHES AND POSITION MOVEMENT YET *****/
    // elevatorCarriageMotor.setInverted(true);
  }

  public void runCarriageUntilReverseLimit() {
    if (returnCarriageReverseLimitSwitchState() == 0) {
      runCarriage(-0.1);
    } else {
      stopCarriage();
      resetCarriageSensorPosition();
    }
    /***** HAVE NOT TESTED LIMIT SWITCHES AND POSITION MOVEMENT YET *****/
    // elevatorCarriageMotor.setInverted(false);
  }

  public void runCarriageToPosition(double pos) {
    /***** NO ACTIVE ENCODER YET *****/
    elevatorCarriageMotor.set(
        ControlMode.MotionMagic, pos, DemandType.ArbitraryFeedForward, getCarriageFeedForward());
  }

  public double getElevatorCarriagePosition() {
    setPos(elevatorCarriageMotor.getSelectedSensorPosition());
    return elevatorCarriageMotor.getSelectedSensorPosition();
  }

  private void setPos(double p) {
    pos = p;
  }

  public static double getPos() {
    return pos;
  }

  public double getElevatorCarriageCurrent() {
    return elevatorCarriageMotor.getStatorCurrent();
  }

  public double getElevatorCarriageTemp() {
    return elevatorCarriageMotor.getTemperature();
  }

  // FIXME: SET CORRECT POSITIONS FOR ELEVATOR MOTORS
  public void runToDownPosition() {
    runCarriageToPosition(0);
  }

  public void runToGroundScorePosition() {
    runCarriageToPosition(-9);
  }

  public void runToHumanPickUpPosition() {
    runCarriageToPosition(-9);
  }

  public void runToTravelPosition() {
    runCarriageToPosition(-10500);
  }

  public void runToLowScorePosition() {
    runCarriageToPosition(-46000);
  }

  public void runToHighScorePosition() {
    runCarriageToPosition(-64400);
  }

  // return the angle of the arm based on the current encoder value
  public double getCarriageAngle() {
    double currentPosition = getElevatorCarriagePosition();

    // divide the encoder position when arm is horizontal by the angle displacement
    // so if you moved the arm 30 degrees and read 1000 ticks, it would be 1000/30 ticks per degree
    // subtract horizontalAngleDisplacement to make the horizontal postion 0 degrees
    // FIXME: Set ticksPerDegree for Elevator.
    double ticksPerDegree = -64400.0 / 170.0;
    double angle = currentPosition / ticksPerDegree - 0;
    return angle;
  }

  public double getCarriageFeedForward() {

    // get the radians of the arm
    // getAngle() returns degrees
    double theta = Math.toRadians(getCarriageAngle());

    SmartDashboard.putNumber("Elevator Carriage Angle", getCarriageAngle());

    // get a range of 0 to 1 to multiply by feedforward.
    // when in horizontal position, value should be 1
    // when in vertical up or down position, value should be 0
    double gravityCompensation = Math.cos(theta);

    SmartDashboard.putNumber("Elevator Carriage Gravity Compensation", gravityCompensation);

    // horizontalHoldOutput is the minimum power required to hold the arm up when horizontal
    // this is a range of 0-1, in our case it was .125 throttle required to keep the arm up
    double feedForward = gravityCompensation * 0.2; // <- horizontalHoldOutput

    SmartDashboard.putNumber("Elevator Carriage Feed Forward", feedForward);

    return feedForward;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
