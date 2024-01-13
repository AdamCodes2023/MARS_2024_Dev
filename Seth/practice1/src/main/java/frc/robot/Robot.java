// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  private final WPI_TalonSRX m_frontleftMotor = new WPI_TalonSRX(12);
  private final WPI_TalonSRX m_frontrightMotor = new WPI_TalonSRX(10);
  private final WPI_TalonSRX m_backleftMotor = new WPI_TalonSRX(13);
  private final WPI_TalonSRX m_backrightMotor = new WPI_TalonSRX(11);
  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_frontrightMotor, m_backrightMotor);
  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_frontleftMotor, m_backleftMotor);
  private final DifferentialDrive m_robotDrive =
      new DifferentialDrive(m_leftMotors, m_rightMotors);
  private final Joystick m_stick = new Joystick(0);

  public Robot() {
    SendableRegistry.addChild(m_robotDrive, m_leftMotors);
    SendableRegistry.addChild(m_robotDrive, m_rightMotors);
  }

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotors.setInverted(true);
  }

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    m_robotDrive.arcadeDrive(-m_stick.getY(), -m_stick.getX());
  }
}
