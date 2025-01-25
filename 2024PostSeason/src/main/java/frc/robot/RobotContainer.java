// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team3061.gyro.GyroIO;
import frc.lib.team3061.gyro.GyroIOPigeon2;
import frc.lib.team3061.swerve.SwerveModule;
import frc.lib.team3061.swerve.SwerveModuleIO;
import frc.lib.team3061.swerve.SwerveModuleIOSim;
import frc.lib.team3061.swerve.SwerveModuleIOTalonFX;
import frc.robot.Constants.Mode;
import frc.robot.commands.CenterOnAprilTagAuto;
import frc.robot.commands.DriveXMeters;
import frc.robot.commands.DriveYMeters;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeInAuto;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.ShootAmp;
import frc.robot.commands.ShootSpeaker;
import frc.robot.commands.ShootSpeakerAuto;
import frc.robot.commands.Stop;
import frc.robot.commands.StopShootSpeakerAuto;
import frc.robot.commands.TeleopSwerve;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.lift.Lift;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter_angle.ShooterAngle;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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
  private Lift lift;
  // private OrchestraPlayer orchestraPlayer;
  private Shooter shooter;
  private ShooterAngle shooterAngle;
  // private Vision vision;

  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to ensure accurate logging
  //private final LoggedDashboardChooser<Command> autoChooser =
      //new LoggedDashboardChooser<>("Auto Routine");

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

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
            intake = new Intake();
            //intake = null;
            lift = new Lift();
            // orchestraPlayer = new OrchestraPlayer();
            shooter = new Shooter();
            shooterAngle = new ShooterAngle();
            // vision = new Vision();
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
            intake = new Intake();
            //intake = null;
            lift = new Lift();
            // orchestraPlayer = new OrchestraPlayer();
            shooter = new Shooter();
            shooterAngle = new ShooterAngle();
            // vision = new Vision();
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
      intake = new Intake();
      //intake = null;
      lift = new Lift();
      // orchestraPlayer = new OrchestraPlayer();
      shooter = new Shooter();
      shooterAngle = new ShooterAngle();
      // vision = new Vision();
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

    oi.getTurboModeButton().onFalse(drivetrain.getDefaultCommand());

    oi.getSlowModeButton()
        .onTrue(
            new TeleopSwerve(
                drivetrain, oi::getSlowTranslateX, oi::getSlowTranslateY, oi::getSlowRotate));

    oi.getSlowModeButton().onFalse(drivetrain.getDefaultCommand());


    oi.getEmergencyTurnRightButton()
        .onTrue(
            new TeleopSwerve(
                drivetrain, oi::getNormalTranslateX, oi::getNormalTranslateY, oi::getEmergencyRightRotate));

    oi.getEmergencyTurnRightButton().onFalse(drivetrain.getDefaultCommand());
    
    oi.getEmergencyTurnLeftButton()
        .onTrue(
            new TeleopSwerve(
                drivetrain, oi::getNormalTranslateX, oi::getNormalTranslateY, oi::getEmergencyLeftRotate));

    oi.getEmergencyTurnLeftButton().onFalse(drivetrain.getDefaultCommand());        

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

    // run intake
    // oi.getIntakeButton().onTrue(Commands.run(() -> intake.runIntake(oi.getIntakeSpeed()),
    // intake));
    // oi.getIntakeButton().onFalse(Commands.runOnce(() -> intake.stopIntake(), intake));
    // run outake
    // oi.getOutakeButton().onTrue(Commands.run(() -> intake.runIntake(oi.getOutakeSpeed()),
    // intake));
    // oi.getOutakeButton().onFalse(Commands.runOnce(() -> intake.stopIntake(), intake));

    /*
    // run positive shooter angle
    oi.getPositiveAngleButton()
        .onTrue(Commands.runOnce(() -> shooterAngle.smartMotionPosition(181.3/360.0), shooterAngle));
    oi.getPositiveAngleButton()
        .onFalse(Commands.runOnce(() -> shooterAngle.stopMotor(), shooterAngle));
    // run negative shooter angle
    oi.getNegativeAngleButton()
        .onTrue(Commands.runOnce(() -> shooterAngle.smartMotionPosition(256.0/360.0), shooterAngle));
    oi.getNegativeAngleButton()
        .onFalse(new FunctionalCommand(shooterAngle::stopMotor, shooterAngle::intakePosition, shooterAngle::stopMotorWithInterrupt, shooterAngle::atIntakeAlignment));
    */

    RepeatCommand repeatShootLights = new RepeatCommand(Commands.runOnce(Lights::turnShoot));

    Command intakeInCommand = new IntakeIn(index, intake, shooterAngle);
    Command intakeOutCommand = new IntakeOut(index, intake, shooterAngle);
    RepeatCommand repeatIn = new RepeatCommand(Commands.runOnce(() -> intakeInCommand.execute()));
    RepeatCommand repeatOut = new RepeatCommand(Commands.runOnce(() -> intakeOutCommand.execute()));
    // INTAKE TESTING
    oi.getIntakeButton().onTrue(repeatIn);
    oi.getIntakeButton().onFalse(Commands.runOnce(() -> repeatIn.cancel()));
    oi.getIntakeButton().onFalse(Commands.runOnce(() -> intakeInCommand.end(true)));

    oi.getOutakeButton().onTrue(repeatOut);
    oi.getOutakeButton().onFalse(Commands.runOnce(() -> repeatOut.cancel()));
    oi.getOutakeButton().onFalse(Commands.runOnce(() -> intakeOutCommand.end(true)));


/*
    // SHOOTER TUNING
    oi.getShootAmpButton().onTrue(Commands.runOnce(() -> shooterAngle.ampPosition()));
    oi.getShootAmpButton().onFalse(new FunctionalCommand(
                shooterAngle::stopMotor,
                shooterAngle::intakePosition,
                shooterAngle::stopMotorWithInterrupt,
                shooterAngle::atIntakeAlignment));

    oi.getShootSpeakerButton().onTrue(Commands.runOnce(() -> shooterAngle.setPosition(179.6)));
    oi.getShootSpeakerButton().onFalse(Commands.runOnce(() -> shooterAngle.stopMotor()));

    oi.getShootSpeakerMidButton().onTrue(Commands.runOnce(() -> shooterAngle.setPosition(199.1)));
    oi.getShootSpeakerMidButton().onFalse(new FunctionalCommand(
                shooterAngle::stopMotor,
                shooterAngle::intakePosition,
                shooterAngle::stopMotorWithInterrupt,
                shooterAngle::atIntakeAlignment));
    
    oi.getShootSpeakerFarButton().onTrue(Commands.runOnce(() -> shooterAngle.setPosition(219.1)));
    oi.getShootSpeakerFarButton().onFalse(new FunctionalCommand(
                shooterAngle::stopMotor,
                shooterAngle::intakePosition,
                shooterAngle::stopMotorWithInterrupt,
                shooterAngle::atIntakeAlignment));
*/



    // SHOOTER TESTING
    Command shootAmpCommand = new ShootAmp(index, shooterAngle, shooter);
    RepeatCommand repeatShootAmp =
        new RepeatCommand(Commands.runOnce(() -> shootAmpCommand.execute()));
    oi.getShootAmpButton().onTrue(repeatShootAmp);
    oi.getShootAmpButton().onFalse(Commands.runOnce(() -> repeatShootAmp.cancel()));
    oi.getShootAmpButton().onFalse(Commands.runOnce(() -> shootAmpCommand.end(true)));
    oi.getShootAmpButton()
        .onFalse(
            new FunctionalCommand(
                shooterAngle::stopMotor,
                shooterAngle::intakePosition,
                shooterAngle::stopMotorWithInterrupt,
                shooterAngle::atIntakeAlignment));
    oi.getShootAmpButton().onTrue(repeatShootLights);
    oi.getShootAmpButton().onFalse(Commands.runOnce(() -> repeatShootLights.cancel()));
    oi.getShootAmpButton().onFalse(Commands.runOnce(Lights::turnOff));

    Command shootSpeakerCommand = new ShootSpeaker(index, shooterAngle, shooter, 179.6, 15000);
    RepeatCommand repeatShootSpeaker =
        new RepeatCommand(Commands.runOnce(() -> shootSpeakerCommand.execute()));
    oi.getShootSpeakerButton().onTrue(repeatShootSpeaker);
    oi.getShootSpeakerButton().onFalse(Commands.runOnce(() -> repeatShootSpeaker.cancel()));
    oi.getShootSpeakerButton().onFalse(Commands.runOnce(() -> shootSpeakerCommand.end(true)));
    oi.getShootSpeakerButton()
        .onFalse(
            new FunctionalCommand(
                shooterAngle::stopMotor,
                shooterAngle::intakePosition,
                shooterAngle::stopMotorWithInterrupt,
                shooterAngle::atIntakeAlignment));
    oi.getShootSpeakerButton().onTrue(repeatShootLights);
    oi.getShootSpeakerButton().onFalse(Commands.runOnce(() -> repeatShootLights.cancel()));
    oi.getShootSpeakerButton().onFalse(Commands.runOnce(Lights::turnOff));

    Command shootSpeakerMidCommand = new ShootSpeaker(index, shooterAngle, shooter, 198.1, 30000);
    RepeatCommand repeatShootSpeakerMid =
        new RepeatCommand(Commands.runOnce(() -> shootSpeakerMidCommand.execute()));
    oi.getShootSpeakerMidButton().onTrue(repeatShootSpeakerMid);
    oi.getShootSpeakerMidButton().onFalse(Commands.runOnce(() -> repeatShootSpeakerMid.cancel()));
    oi.getShootSpeakerMidButton().onFalse(Commands.runOnce(() -> shootSpeakerMidCommand.end(true)));
    oi.getShootSpeakerMidButton()
        .onFalse(
            new FunctionalCommand(
                shooterAngle::stopMotor,
                shooterAngle::intakePosition,
                shooterAngle::stopMotorWithInterrupt,
                shooterAngle::atIntakeAlignment));
    oi.getShootSpeakerMidButton().onTrue(repeatShootLights);
    oi.getShootSpeakerMidButton().onFalse(Commands.runOnce(() -> repeatShootLights.cancel()));
    oi.getShootSpeakerMidButton().onFalse(Commands.runOnce(Lights::turnOff));

    Command shootSpeakerFarCommand = new ShootSpeaker(index, shooterAngle, shooter, 219.1, 30000);
    RepeatCommand repeatShootSpeakerFar =
        new RepeatCommand(Commands.runOnce(() -> shootSpeakerFarCommand.execute()));
    oi.getShootSpeakerFarButton().onTrue(repeatShootSpeakerFar);
    oi.getShootSpeakerFarButton().onFalse(Commands.runOnce(() -> repeatShootSpeakerFar.cancel()));
    oi.getShootSpeakerFarButton().onFalse(Commands.runOnce(() -> shootSpeakerFarCommand.end(true)));
    oi.getShootSpeakerFarButton()
        .onFalse(
            new FunctionalCommand(
                shooterAngle::stopMotor,
                shooterAngle::intakePosition,
                shooterAngle::stopMotorWithInterrupt,
                shooterAngle::atIntakeAlignment));
    oi.getShootSpeakerFarButton().onTrue(repeatShootLights);
    oi.getShootSpeakerFarButton().onFalse(Commands.runOnce(() -> repeatShootLights.cancel()));
    oi.getShootSpeakerFarButton().onFalse(Commands.runOnce(Lights::turnOff));
    

    oi.getShootForwardButton().onTrue(Commands.runOnce(() -> shooter.runShooter(10000.0)));
    oi.getShootForwardButton().onFalse(Commands.runOnce(() -> shooter.runShooter(0.0)));
    oi.getShootForwardButton().onTrue(Commands.runOnce(() -> index.runIndex(-0.7)));
    oi.getShootForwardButton().onFalse(Commands.runOnce(() -> index.stopIndex()));
    oi.getShootForwardButton().onTrue(repeatShootLights);
    oi.getShootForwardButton().onFalse(Commands.runOnce(() -> repeatShootLights.cancel()));
    oi.getShootForwardButton().onFalse(Commands.runOnce(Lights::turnOff));

    oi.getShootReverseButton().onTrue(Commands.runOnce(() -> shooter.runShooter(-10000.0)));
    oi.getShootReverseButton().onFalse(Commands.runOnce(() -> shooter.runShooter(0.0)));
    oi.getShootReverseButton().onTrue(Commands.runOnce(() -> index.runIndex(0.7)));
    oi.getShootReverseButton().onFalse(Commands.runOnce(() -> index.stopIndex()));
    oi.getShootReverseButton().onTrue(repeatShootLights);
    oi.getShootReverseButton().onFalse(Commands.runOnce(() -> repeatShootLights.cancel()));
    oi.getShootReverseButton().onFalse(Commands.runOnce(Lights::turnOff));


    //ADJUSTMENT SWITCHES
    oi.getPositiveAdjustSpeedButton().onTrue(Commands.runOnce(() -> shooter.increaseSpeedCorrector()));

    oi.getNegativeAdjustSpeedButton().onTrue(Commands.runOnce(() -> shooter.decreaseSpeedCorrector()));

    oi.getPositiveAdjustAngleButton().onTrue(Commands.runOnce(() -> shooterAngle.increaseAngleCorrector()));

    oi.getNegativeAdjustAngleButton().onTrue(Commands.runOnce(() -> shooterAngle.decreaseAngleCorrector()));


    // CENTER ON APRILTAGS
    oi.getAprilTagButton().onTrue(new RepeatCommand(Commands.runOnce(() -> drivetrain.centerOnAprilTag(), drivetrain)));
    oi.getAprilTagButton().onFalse(drivetrain.getDefaultCommand());


    // LIFT TESTING
    oi.getLiftDownButton().onTrue(Commands.runOnce(() -> lift.runClimbers(0.5)));
    oi.getLiftDownButton().onFalse(Commands.runOnce(() -> lift.runClimbers(0.0)));

    oi.getLiftUpButton().onTrue(Commands.runOnce(() -> lift.runClimbers(-0.5)));
    oi.getLiftUpButton().onFalse(Commands.runOnce(() -> lift.runClimbers(0.0)));

    /* LIGHT TESTING */
    // oi.getBlueLightButton().onTrue(Commands.runOnce(Lights::turnBlue));
    // oi.getBlueLightButton().onFalse(Commands.runOnce(Lights::turnOff));

    // oi.getRedLightButton().onTrue(Commands.runOnce(Lights::turnRed));
    // oi.getRedLightButton().onFalse(Commands.runOnce(Lights::turnOff));

    // oi.getShootLightButton().onTrue(Commands.runOnce(Lights::turnShoot));
    // oi.getShootLightButton().onFalse(Commands.runOnce(Lights::turnOff));

    // oi.getMarsLightButton().onTrue(Commands.runOnce(Lights::turnMars));
    // oi.getMarsLightButton().onFalse(Commands.runOnce(Lights::turnOff));

    // oi.getBlueIntakeLightButton().onTrue(Commands.runOnce(Lights::turnIntakeBlue));
    // oi.getBlueIntakeLightButton().onFalse(Commands.runOnce(Lights::turnOff));

    // oi.getRedIntakeLightButton().onTrue(Commands.runOnce(Lights::turnIntakeRed));
    // oi.getRedIntakeLightButton().onFalse(Commands.runOnce(Lights::turnOff));
  }

  /** Use this method to define your commands for autonomous mode. */
  // FIXME: ADD EVENTS FOR AUTONOMOUS PATHS.
  private void configureAutoCommands() {
    Command testPath = new PathPlannerAuto("New Auto");
    ShootSpeakerAuto shootSpeakerAuto = new ShootSpeakerAuto(intake, index, shooterAngle, shooter, 179.6, 15000);
    DriveXMeters driveXMeters = new DriveXMeters(drivetrain, 4, true);
    StopShootSpeakerAuto stopShootSpeakerAuto = new StopShootSpeakerAuto(index, shooter);
    FunctionalCommand lowerAngle = new FunctionalCommand(shooterAngle::stopMotor, shooterAngle::intakePosition, shooterAngle::stopMotorWithInterrupt, shooterAngle::atIntakeAlignment);
    
/* 
    SequentialCommandGroup scoreAndLeave = new SequentialCommandGroup(
    //new CenterOnAprilTagAuto(drivetrain), 
    shootSpeakerAuto,
    driveXMeters,
    stopShootSpeakerAuto,
    lowerAngle);
*/     
    // Command path1 = new PathPlannerAuto("Path1");
    // Command path2 = new PathPlannerAuto("Path2");
    // Command path3 = new PathPlannerAuto("Path3");
    // Command path4 = new PathPlannerAuto("Path4");
    // Command path5 = new PathPlannerAuto("Path5");

    

    autoChooser.setDefaultOption("DO NOTHING", new Stop(drivetrain));
    autoChooser.addOption("Test Path", testPath);
    autoChooser.addOption("Score and Leave X", Commands.sequence(new ProxyCommand(new ShootSpeakerAuto(intake, index, shooterAngle, shooter, 179.6, 15000)), new ProxyCommand(new DriveXMeters(drivetrain, -5.0, true)), new ProxyCommand(new StopShootSpeakerAuto(index, shooter)), new ProxyCommand(new FunctionalCommand(shooterAngle::stopMotor, shooterAngle::intakePosition, shooterAngle::stopMotorWithInterrupt, shooterAngle::atIntakeAlignment))));
    autoChooser.addOption("Score and Leave X FAR", Commands.sequence(new ProxyCommand(new ShootSpeakerAuto(intake, index, shooterAngle, shooter, 179.6, 15000)), new ProxyCommand(new DriveXMeters(drivetrain, -15.0, true)), new ProxyCommand(new StopShootSpeakerAuto(index, shooter)), new ProxyCommand(new FunctionalCommand(shooterAngle::stopMotor, shooterAngle::intakePosition, shooterAngle::stopMotorWithInterrupt, shooterAngle::atIntakeAlignment))));
    autoChooser.addOption("Score and Leave Y POSITIVE", Commands.sequence(new ProxyCommand(new ShootSpeakerAuto(intake, index, shooterAngle, shooter, 179.6, 15000)), new ProxyCommand(new DriveXMeters(drivetrain, -15.0, true)), new ProxyCommand(new DriveYMeters(drivetrain, 5.0, true)), new ProxyCommand(new StopShootSpeakerAuto(index, shooter)), new ProxyCommand(new FunctionalCommand(shooterAngle::stopMotor, shooterAngle::intakePosition, shooterAngle::stopMotorWithInterrupt, shooterAngle::atIntakeAlignment))));
    autoChooser.addOption("Score and Leave Y NEGATIVE", Commands.sequence(new ProxyCommand(new ShootSpeakerAuto(intake, index, shooterAngle, shooter, 179.6, 15000)), new ProxyCommand(new DriveXMeters(drivetrain, -15.0, true)), new ProxyCommand(new DriveYMeters(drivetrain, -5.0, true)), new ProxyCommand(new StopShootSpeakerAuto(index, shooter)), new ProxyCommand(new FunctionalCommand(shooterAngle::stopMotor, shooterAngle::intakePosition, shooterAngle::stopMotorWithInterrupt, shooterAngle::atIntakeAlignment))));
    autoChooser.addOption("Score and Intake", Commands.sequence(new ProxyCommand(new ShootSpeakerAuto(intake, index, shooterAngle, shooter, 179.6, 15000)), new ProxyCommand(new DriveXMeters(drivetrain, -3.0, true)), new ProxyCommand(new StopShootSpeakerAuto(index, shooter)), new ProxyCommand(new FunctionalCommand(shooterAngle::stopMotor, shooterAngle::intakePosition, shooterAngle::stopMotorWithInterrupt, shooterAngle::atIntakeAlignment)), new ProxyCommand(new ParallelCommandGroup(new IntakeInAuto(index, intake, shooterAngle), new DriveXMeters(drivetrain, -6.0, true))), new ProxyCommand(new DriveXMeters(drivetrain, 0.0, true)), new ProxyCommand(new ShootSpeakerAuto(intake, index, shooterAngle, shooter, 179.6, 15000))));
    autoChooser.addOption("Score and Sit", Commands.sequence(new ProxyCommand(new ShootSpeakerAuto(intake, index, shooterAngle, shooter, 179.6, 15000))));
    autoChooser.addOption("Score and Slide POSITIVE", Commands.sequence(new ProxyCommand(new ShootSpeakerAuto(intake, index, shooterAngle, shooter, 179.6, 15000)), new ProxyCommand(new DriveYMeters(drivetrain, 15.0, true)), new ProxyCommand(new DriveXMeters(drivetrain, -10.0, true)), new ProxyCommand(new StopShootSpeakerAuto(index, shooter)), new ProxyCommand(new FunctionalCommand(shooterAngle::stopMotor, shooterAngle::intakePosition, shooterAngle::stopMotorWithInterrupt, shooterAngle::atIntakeAlignment))));
    autoChooser.addOption("Score and Slide NEGATIVE", Commands.sequence(new ProxyCommand(new ShootSpeakerAuto(intake, index, shooterAngle, shooter, 179.6, 15000)), new ProxyCommand(new DriveYMeters(drivetrain, -15.0, true)), new ProxyCommand(new DriveXMeters(drivetrain, -10.0, true)), new ProxyCommand(new StopShootSpeakerAuto(index, shooter)), new ProxyCommand(new FunctionalCommand(shooterAngle::stopMotor, shooterAngle::intakePosition, shooterAngle::stopMotorWithInterrupt, shooterAngle::atIntakeAlignment))));


    // autoChooser.addOption("Path1", path1);
    // autoChooser.addOption("Path2", path2);
    // autoChooser.addOption("Path3", path3);
    // autoChooser.addOption("Path4", path4);
    // autoChooser.addOption("Path5", path5);
    SmartDashboard.putData(autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
