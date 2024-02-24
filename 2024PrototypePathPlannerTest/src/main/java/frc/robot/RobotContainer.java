// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team3061.gyro.GyroIO;
import frc.lib.team3061.gyro.GyroIOPigeon2;
import frc.lib.team3061.swerve.SwerveModule;
import frc.lib.team3061.swerve.SwerveModuleIO;
import frc.lib.team3061.swerve.SwerveModuleIOSim;
import frc.lib.team3061.swerve.SwerveModuleIOTalonFX;
import frc.robot.Constants.Mode;
import frc.robot.commands.TeleopSwerve;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.orchestra.OrchestraPlayer;
import frc.robot.subsystems.shooter_angle.ShooterAngle;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private OperatorInterface oi = new OperatorInterface() {};
  private CommandJoystick actionJoystick;

  private Trigger[] actionJoystickButtons;

  private Drivetrain drivetrain;
  private Index index;
  private Intake intake;
  private OrchestraPlayer orchestraPlayer;
  private ShooterAngle shooterAngle;
  private Vision vision;

  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to ensure accurate logging
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  // RobotContainer singleton
  private static RobotContainer robotContainer = new RobotContainer();

  /** Create the container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // create real, simulated, or replay subsystems based on the mode and robot specified
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case ROBOT_2022_PRESEASON:
          {
            GyroIO gyro = new GyroIOPigeon2(PIGEON_ID);

            SwerveModule flModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        0,
                        FRONT_LEFT_MODULE_DRIVE_MOTOR,
                        FRONT_LEFT_MODULE_STEER_MOTOR,
                        FRONT_LEFT_MODULE_STEER_ENCODER,
                        FRONT_LEFT_MODULE_STEER_OFFSET),
                    0,
                    MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule frModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        1,
                        FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                        FRONT_RIGHT_MODULE_STEER_MOTOR,
                        FRONT_RIGHT_MODULE_STEER_ENCODER,
                        FRONT_RIGHT_MODULE_STEER_OFFSET),
                    1,
                    MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule blModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        2,
                        BACK_LEFT_MODULE_DRIVE_MOTOR,
                        BACK_LEFT_MODULE_STEER_MOTOR,
                        BACK_LEFT_MODULE_STEER_ENCODER,
                        BACK_LEFT_MODULE_STEER_OFFSET),
                    2,
                    MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule brModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        3,
                        BACK_RIGHT_MODULE_DRIVE_MOTOR,
                        BACK_RIGHT_MODULE_STEER_MOTOR,
                        BACK_RIGHT_MODULE_STEER_ENCODER,
                        BACK_RIGHT_MODULE_STEER_OFFSET),
                    3,
                    MAX_VELOCITY_METERS_PER_SECOND);

            drivetrain = new Drivetrain(gyro, flModule, frModule, blModule, brModule);
            index = new Index();
            // intake = new Intake();
            // orchestraPlayer = new OrchestraPlayer();
            shooterAngle = new ShooterAngle();
            //vision = new Vision();
            // NO REV PNEUMATICS HUB YET
            // new Pneumatics(new PneumaticsIORev());
            actionJoystick = new CommandJoystick(1);
            actionJoystickButtons = new Trigger[13];

            for (int index = 1; index < actionJoystickButtons.length; index++) {
              actionJoystickButtons[index] = actionJoystick.button(index);
            }
            break;
          }
        case ROBOT_SIMBOT:
          {
            SwerveModule flModule =
                new SwerveModule(new SwerveModuleIOSim(), 0, MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule frModule =
                new SwerveModule(new SwerveModuleIOSim(), 1, MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule blModule =
                new SwerveModule(new SwerveModuleIOSim(), 2, MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule brModule =
                new SwerveModule(new SwerveModuleIOSim(), 3, MAX_VELOCITY_METERS_PER_SECOND);
            drivetrain = new Drivetrain(new GyroIO() {}, flModule, frModule, blModule, brModule);
            index = new Index();
            // intake = new Intake();
            // orchestraPlayer = new OrchestraPlayer();
            shooterAngle = new ShooterAngle();
            //vision = new Vision();
            // NO REV PNEUMATICS HUB YET
            // new Pneumatics(new PneumaticsIO() {});

            actionJoystick = new CommandJoystick(1);
            actionJoystickButtons = new Trigger[13];

            for (int index = 1; index < actionJoystickButtons.length; index++) {
              actionJoystickButtons[index] = actionJoystick.button(index);
            }

            break;
          }
        default:
          break;
      }

    } else {
      SwerveModule flModule =
          new SwerveModule(new SwerveModuleIO() {}, 0, MAX_VELOCITY_METERS_PER_SECOND);

      SwerveModule frModule =
          new SwerveModule(new SwerveModuleIO() {}, 1, MAX_VELOCITY_METERS_PER_SECOND);

      SwerveModule blModule =
          new SwerveModule(new SwerveModuleIO() {}, 2, MAX_VELOCITY_METERS_PER_SECOND);

      SwerveModule brModule =
          new SwerveModule(new SwerveModuleIO() {}, 3, MAX_VELOCITY_METERS_PER_SECOND);
      drivetrain = new Drivetrain(new GyroIO() {}, flModule, frModule, blModule, brModule);
      index = new Index();
      // intake = new Intake();
      // orchestraPlayer = new OrchestraPlayer();
      shooterAngle = new ShooterAngle();
      //vision = new Vision();
      // NO REV PNEUMATICS HUB YET
      // new Pneumatics(new PneumaticsIO() {});
      actionJoystick = new CommandJoystick(1);
      actionJoystickButtons = new Trigger[13];

      for (int index = 1; index < actionJoystickButtons.length; index++) {
        actionJoystickButtons[index] = actionJoystick.button(index);
      }
    }

    // disable all telemetry in the LiveWindow to reduce the processing during each iteration
    LiveWindow.disableAllTelemetry();

    updateOI();

    configureAutoCommands();
  }

  /**
   * This method scans for any changes to the connected joystick. If anything changed, it creates
   * new OI objects and binds all of the buttons to commands.
   */
  public void updateOI() {
    if (!OISelector.didJoysticksChange()) {
      return;
    }

    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    oi = OISelector.findOperatorInterface();

    /*
     * Set up the default command for the drivetrain. The joysticks' values map to percentage of the
     * maximum velocities. The velocities may be specified from either the robot's frame of
     * reference or the field's frame of reference. In the robot's frame of reference, the positive
     * x direction is forward; the positive y direction, left; position rotation, CCW. In the field
     * frame of reference, the origin of the field to the lower left corner (i.e., the corner of the
     * field to the driver's right). Zero degrees is away from the driver and increases in the CCW
     * direction. This is why the left joystick's y axis specifies the velocity in the x direction
     * and the left joystick's x axis specifies the velocity in the y direction.
     */
    drivetrain.setDefaultCommand(
        new TeleopSwerve(
            drivetrain, oi::getNormalTranslateX, oi::getNormalTranslateY, oi::getNormalRotate));

    configureButtonBindings();
  }

  /**
   * Factory method to create the singleton robot container object.
   *
   * @return the singleton robot container object
   */
  public static RobotContainer getInstance() {
    return robotContainer;
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {
    // turbo and slow modes for the drivetrain.
    oi.getTurboModeButton()
        .onTrue(
            new TeleopSwerve(
                drivetrain, oi::getTurboTranslateX, oi::getTurboTranslateY, oi::getTurboRotate));

    oi.getTurboModeButton()
        .onFalse(drivetrain.getDefaultCommand());


    oi.getSlowModeButton()
        .onTrue(
            new TeleopSwerve(
                drivetrain, oi::getSlowTranslateX, oi::getSlowTranslateY, oi::getSlowRotate));
                
    oi.getSlowModeButton()
        .onFalse(drivetrain.getDefaultCommand());


    // field-relative toggle
    oi.getFieldRelativeButton()
        .toggleOnTrue(
            Commands.either(
                Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
                Commands.runOnce(drivetrain::enableFieldRelative, drivetrain),
                drivetrain::getFieldRelative));

    // reset gyro to 0 degrees
    oi.getResetGyroButton().onTrue(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain));

    // reset pitch to 0 degrees
    oi.getResetPitchButton().onTrue(Commands.runOnce(drivetrain::zeroPitch, drivetrain));

    // x-stance
    oi.getXStanceButton().onTrue(Commands.runOnce(drivetrain::enableXstance, drivetrain));
    oi.getXStanceButton().onFalse(Commands.runOnce(drivetrain::disableXstance, drivetrain));

    // music
    /*
    oi.getPlayMusicButton().onTrue(Commands.runOnce(orchestraPlayer::playMusic, orchestraPlayer));
    oi.getPlayMusicButton().onFalse(Commands.runOnce(orchestraPlayer::stopMusic, orchestraPlayer));
    */

    /*
    // run intake
    oi.getIntakeButton().onTrue(Commands.run(() -> intake.runIntake(oi.getIntakeSpeed()), intake));
    oi.getIntakeButton().onFalse(Commands.runOnce(() -> intake.stopIntake(), intake));
    // run outake
    oi.getOutakeButton().onTrue(Commands.run(() -> intake.runIntake(oi.getOutakeSpeed()), intake));
    oi.getOutakeButton().onFalse(Commands.runOnce(() -> intake.stopIntake(), intake));
    */

    // run positive shooter angle
    // oi.getPositiveAngleButton().onTrue(Commands.runOnce(() -> shooterAngle.setMotorSpeed(0.5),
    // shooterAngle));
    oi.getPositiveAngleButton()
        .onTrue(Commands.runOnce(() -> shooterAngle.smartMotionPosition(0.7), shooterAngle));
    oi.getPositiveAngleButton()
        .onFalse(Commands.runOnce(() -> shooterAngle.stopMotor(), shooterAngle));
    // run negative shooter angle
    // oi.getNegativeAngleButton().onTrue(Commands.runOnce(() -> shooterAngle.setMotorSpeed(-0.5),
    // shooterAngle));
    oi.getNegativeAngleButton()
        .onTrue(Commands.runOnce(() -> shooterAngle.smartMotionPosition(0.2), shooterAngle));
    oi.getNegativeAngleButton()
        .onFalse(Commands.runOnce(() -> shooterAngle.stopMotor(), shooterAngle));


    // center on apriltags
    oi.getAprilTagButton()
        .onTrue(Commands.run(() -> drivetrain.centerOnAprilTag(oi::getNormalTranslateX, oi::getNormalTranslateY)));
    oi.getAprilTagButton()
        .onFalse(drivetrain.getDefaultCommand());


    /* LIGHT TESTING */
    oi.getBlueLightButton().onTrue(Commands.runOnce(Lights::turnBlue));
    oi.getBlueLightButton().onFalse(Commands.runOnce(Lights::turnOff));

    oi.getRedLightButton().onTrue(Commands.runOnce(Lights::turnRed));
    oi.getRedLightButton().onFalse(Commands.runOnce(Lights::turnOff));

    oi.getShootLightButton().onTrue(Commands.runOnce(Lights::turnShoot));
    oi.getShootLightButton().onFalse(Commands.runOnce(Lights::turnOff));

    oi.getMarsLightButton().onTrue(Commands.runOnce(Lights::turnMars));
    oi.getMarsLightButton().onFalse(Commands.runOnce(Lights::turnOff));

    oi.getBlueIntakeLightButton().onTrue(Commands.runOnce(Lights::turnIntakeBlue));
    oi.getBlueIntakeLightButton().onFalse(Commands.runOnce(Lights::turnOff));

    oi.getRedIntakeLightButton().onTrue(Commands.runOnce(Lights::turnIntakeRed));
    oi.getRedIntakeLightButton().onFalse(Commands.runOnce(Lights::turnOff));
  }

  /** Use this method to define your commands for autonomous mode. */
  // FIXME: ADD EVENTS FOR AUTONOMOUS PATHS.
  private void configureAutoCommands() {
    Command testPath = new PathPlannerAuto("New Auto");
    //Command path1 = new PathPlannerAuto("Path1");
    //Command path2 = new PathPlannerAuto("Path2");
    //Command path3 = new PathPlannerAuto("Path3");
    //Command path4 = new PathPlannerAuto("Path4");
    //Command path5 = new PathPlannerAuto("Path5");

    autoChooser.addOption("Test Path", testPath);
    //autoChooser.addOption("Path1", path1);
    //autoChooser.addOption("Path2", path2);
    //autoChooser.addOption("Path3", path3);
    //autoChooser.addOption("Path4", path4);
    //autoChooser.addOption("Path5", path5);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
