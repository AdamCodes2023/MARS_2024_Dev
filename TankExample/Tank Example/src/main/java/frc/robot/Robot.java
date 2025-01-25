// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.RelayForward;
import frc.robot.commands.RelayReverse;
import frc.robot.subsystems.RelaySubsystem;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_robotDrive;
  private Joystick m_leftStick;
  private Joystick m_rightStick;

  private JoystickButton m_relayButtonFwd;
  private JoystickButton m_relayButtonRev;

  private RelaySubsystem m_subsystem;

  private RelayForward forwardCommand;
  private RelayReverse reverseCommand;

  private final WPI_TalonSRX m_leftFrontMotor = new WPI_TalonSRX(12);
  private final WPI_TalonSRX m_rightFrontMotor = new WPI_TalonSRX(10);
  private final WPI_TalonSRX m_leftRearMotor = new WPI_TalonSRX(13);
  private final WPI_TalonSRX m_rightRearMotor = new WPI_TalonSRX(11);

  @Override
  public void robotInit() {
    SendableRegistry.addChild(m_robotDrive, m_leftFrontMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightFrontMotor);
    SendableRegistry.addChild(m_robotDrive, m_leftRearMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightRearMotor);

    m_leftFrontMotor.configFactoryDefault();
    m_rightFrontMotor.configFactoryDefault();
    m_leftRearMotor.configFactoryDefault();
    m_leftRearMotor.configFactoryDefault();

    // Invert one side of the drivetrain
    m_leftFrontMotor.setInverted(false);
    m_leftRearMotor.setInverted(false);
    m_rightFrontMotor.setInverted(true);
    m_rightRearMotor.setInverted(true);

    // Add followers
    m_leftRearMotor.follow(m_leftFrontMotor);
    m_rightRearMotor.follow(m_rightFrontMotor);

    // Construct drivetrain
    m_robotDrive = new DifferentialDrive(m_leftFrontMotor::set, m_rightFrontMotor::set);
    // Function undefined? m_robotDrive.setRightSideInverted();

    // Assign control
    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);

    // Initialize subsystems
    m_subsystem = new RelaySubsystem();

    // Initialize commands
    forwardCommand = new RelayForward(m_subsystem);
    reverseCommand = new RelayReverse(m_subsystem);

    // Assign relay, buttons
    m_relayButtonFwd = new JoystickButton(m_rightStick, 11);
    m_relayButtonRev = new JoystickButton(m_rightStick, 10);
      
  }

  @Override
  public void teleopPeriodic() {
    // Apply joystick input to drive system
    m_robotDrive.tankDrive(-m_leftStick.getY(), -m_rightStick.getY());

    // Listen for fwd/rev command input
    m_relayButtonFwd.whileTrue(forwardCommand);
    m_relayButtonRev.whileTrue(reverseCommand);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
