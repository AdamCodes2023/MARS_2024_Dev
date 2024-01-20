// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team3061.gyro.GyroIO;
import frc.lib.team3061.gyro.GyroIOPigeon2;
import frc.lib.team3061.swerve.SwerveModule;
import frc.lib.team3061.swerve.SwerveModuleIO;
import frc.lib.team3061.swerve.SwerveModuleIOSim;
import frc.lib.team3061.swerve.SwerveModuleIOTalonFX;
import frc.lib.team3061.vision.Vision;
import frc.lib.team3061.vision.VisionConstants;
import frc.lib.team3061.vision.VisionIO;
import frc.lib.team3061.vision.VisionIOPhotonVision;
import frc.lib.team3061.vision.VisionIOSim;
import frc.robot.Constants.Mode;
import frc.robot.commands.CenterOnChargingPad;
import frc.robot.commands.DriveXMeters;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.commands.FollowPath;
import frc.robot.commands.PickUpGamePiece;
import frc.robot.commands.PlaceAuton;
import frc.robot.commands.ScoreGamePiece;
import frc.robot.commands.StartingCondition;
import frc.robot.commands.TeleopSwerve;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorCarriage;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.lights.LightsConstants;
import frc.robot.subsystems.pneumaticGripper.GripperWrist;
import frc.robot.subsystems.pneumaticGripper.GripperWristConstants;
import frc.robot.subsystems.pneumaticGripper.MarsPneumatics;
import frc.robot.subsystems.pneumaticGripper.MarsPneumaticsConstants;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
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
  private UsbCamera c = CameraServer.startAutomaticCapture();
  private MjpegServer s = CameraServer.addServer("gripStream");

  private Trigger[] actionJoystickButtons;

  private Drivetrain drivetrain;
  private Elevator elevator;
  private ElevatorCarriage elevatorCarriage;
  private GripperWrist gripperWrist;
  private MarsPneumatics pneumatics;
  private Lights lights;

  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to ensure accurate logging
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  // RobotContainer singleton
  private static RobotContainer robotContainer = new RobotContainer();

  /** Create the container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s.setSource(c);
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
            elevator = new Elevator();
            elevatorCarriage = new ElevatorCarriage();
            gripperWrist = new GripperWrist();
            pneumatics = new MarsPneumatics();
            lights = new Lights();
            // NO REV PNEUMATICS HUB YET
            // new Pneumatics(new PneumaticsIORev());
            new Vision(new VisionIOPhotonVision(CAMERA_NAME));
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
            elevator = new Elevator();
            elevatorCarriage = new ElevatorCarriage();
            gripperWrist = new GripperWrist();
            pneumatics = new MarsPneumatics();
            lights = new Lights();
            // NO REV PNEUMATICS HUB YET
            // new Pneumatics(new PneumaticsIO() {});
            AprilTagFieldLayout layout;
            try {
              layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
            } catch (IOException e) {
              layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
            }
            new Vision(
                new VisionIOSim(layout, drivetrain::getPose, VisionConstants.ROBOT_TO_CAMERA));

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
      elevator = new Elevator();
      elevatorCarriage = new ElevatorCarriage();
      gripperWrist = new GripperWrist();
      pneumatics = new MarsPneumatics();
      lights = new Lights();
      // NO REV PNEUMATICS HUB YET
      // new Pneumatics(new PneumaticsIO() {});
      new Vision(new VisionIO() {});
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
        .onFalse(
            new TeleopSwerve(
                drivetrain, oi::getNormalTranslateX, oi::getNormalTranslateY, oi::getNormalRotate));
    oi.getSlowModeButton()
        .onTrue(
            new TeleopSwerve(
                drivetrain, oi::getSlowTranslateX, oi::getSlowTranslateY, oi::getSlowRotate));
    oi.getSlowModeButton()
        .onFalse(
            new TeleopSwerve(
                drivetrain, oi::getNormalTranslateX, oi::getNormalTranslateY, oi::getNormalRotate));

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

    // center on charging pad (using pitch to detect slope)
    oi.getCenterOnChargingPadButton()
        .onTrue(Commands.run(drivetrain::centerOnChargingPad, drivetrain));
    oi.getCenterOnChargingPadButton().onFalse(Commands.runOnce(drivetrain::stop, drivetrain));

    // center on cone (using Photon Vendor Library to calculate distance)
    // oi.getCenterOnConeButton().onTrue(Commands.run(drivetrain::centerOnCone, drivetrain));
    // oi.getCenterOnConeButton().onFalse(Commands.runOnce(drivetrain::stop, drivetrain));

    // center on cube (using Photon Vendor Library to calculate distance)
    // oi.getCenterOnCubeButton().onTrue(Commands.run(drivetrain::centerOnCube, drivetrain));
    // oi.getCenterOnCubeButton().onFalse(Commands.runOnce(drivetrain::stop, drivetrain));

    // center on object (using Photon Vendor Library to calculate distance)
    oi.getCenterOnObjectButton().onTrue(Commands.run(drivetrain::centerOnObject, drivetrain));
    oi.getCenterOnObjectButton().onFalse(Commands.runOnce(drivetrain::stop, drivetrain));

    // center on low node (using Photon Vendor Library to calculate distance)
    // oi.getCenterOnLowNodeButton().onTrue(Commands.run(drivetrain::centerOnLowNode, drivetrain));
    // oi.getCenterOnLowNodeButton().onFalse(Commands.runOnce(drivetrain::stop, drivetrain));

    // center on mid node (using Photon Vendor Library to calculate distance)
    oi.getCenterOnMidNodeButton().onTrue(Commands.run(drivetrain::centerOnMidNode, drivetrain));
    oi.getCenterOnMidNodeButton().onFalse(Commands.runOnce(drivetrain::stop, drivetrain));

    // center on high node (using Photon Vendor Library to calculate distance)
    oi.getCenterOnHighNodeButton().onTrue(Commands.run(drivetrain::centerOnHighNode, drivetrain));
    oi.getCenterOnHighNodeButton().onFalse(Commands.runOnce(drivetrain::stop, drivetrain));

    // center on April Tags (using Photon Vendor Library to calculate distance)
    oi.getCenterOnAprilTagButton().onTrue(Commands.run(drivetrain::centerOnAprilTag, drivetrain));
    oi.getCenterOnAprilTagButton().onFalse(Commands.runOnce(drivetrain::stop, drivetrain));

    /***** GRIPPER COMMANDS *****/
    actionJoystickButtons[GripperWristConstants.forwardGripperWristSwitchNumber].onTrue(
        Commands.run(gripperWrist::runUntilForwardLimit, gripperWrist));
    actionJoystickButtons[GripperWristConstants.forwardGripperWristSwitchNumber].onFalse(
        Commands.run(gripperWrist::runUntilReverseLimit, gripperWrist));
    // actionJoystickButtons[GripperWristConstants.reverseGripperWristSwitchNumber].onTrue(
    // Commands.run(gripperWrist::runUntilReverseLimit, gripperWrist));
    // actionJoystickButtons[GripperWristConstants.reverseGripperWristSwitchNumber].onFalse(
    // Commands.runOnce(gripperWrist::stop, gripperWrist));
    // actionJoystickButtons[GripperWristConstants.toPosGripperWristButtonNumber].onTrue(
    // Commands.runOnce(() -> gripperWrist.runToPosition(18091), gripperWrist));
    // actionJoystickButtons[GripperWristConstants.toPosGripperWristButtonNumber].onFalse(
    // Commands.runOnce(gripperWrist::stop, gripperWrist));
    // actionJoystickButtons[GripperWristConstants.zThrottleGripperWristButtonNumber].onTrue(
    // Commands.run(() -> gripperWrist.runToPosition((-actionJoystick.getZ() + 1.0) * 18091),
    // gripperWrist));
    // actionJoystickButtons[GripperWristConstants.zThrottleGripperWristButtonNumber].onFalse(
    // Commands.runOnce(gripperWrist::stop, gripperWrist));

    actionJoystickButtons[MarsPneumaticsConstants.solenoidSingle1SwitchNumber].onTrue(
        Commands.runOnce(pneumatics::grip, pneumatics));
    actionJoystickButtons[MarsPneumaticsConstants.solenoidSingle1SwitchNumber].onFalse(
        Commands.runOnce(pneumatics::unGrip, pneumatics));
    actionJoystickButtons[MarsPneumaticsConstants.solenoidSingle3SwitchNumber].onTrue(
        Commands.runOnce(pneumatics::tripDown, pneumatics));
    actionJoystickButtons[MarsPneumaticsConstants.solenoidSingle3SwitchNumber].onFalse(
        Commands.runOnce(pneumatics::tripUp, pneumatics));
    /***** GRIPPER COMMANDS *****/

    /***** ELEVATOR COMMANDS *****/
    // actionJoystickButtons[ElevatorConstants.forwardElevatorButtonNumber].onTrue(
    // Commands.run(elevator::runUntilForwardLimit, elevator));
    // actionJoystickButtons[ElevatorConstants.forwardElevatorButtonNumber].onFalse(
    // Commands.runOnce(elevator::stop, elevator));
    // actionJoystickButtons[ElevatorConstants.forwardElevatorButtonNumber].onTrue(
    // Commands.run(gripperWrist::runToElevatorSamePos, gripperWrist));
    // actionJoystickButtons[ElevatorConstants.forwardElevatorButtonNumber].onTrue(Commands.run(gripperWrist::runToElevatorOppositePos, gripperWrist));
    // actionJoystickButtons[ElevatorConstants.forwardElevatorButtonNumber].onFalse(
    // Commands.runOnce(gripperWrist::stop, gripperWrist));

    actionJoystickButtons[ElevatorConstants.reverseElevatorButtonNumber].onTrue(
        // Commands.run(elevator::runUntilReverseLimit, elevator));
        Commands.run(elevator::runToDownPosition, elevator));
    actionJoystickButtons[ElevatorConstants.reverseElevatorButtonNumber].onFalse(
        Commands.runOnce(elevator::stop, elevator));
    actionJoystickButtons[ElevatorConstants.reverseElevatorButtonNumber].onTrue(
        Commands.run(elevatorCarriage::runCarriageUntilForwardLimit, elevatorCarriage));
    actionJoystickButtons[ElevatorConstants.reverseElevatorButtonNumber].onFalse(
        Commands.runOnce(elevatorCarriage::stopCarriage, elevatorCarriage));
    actionJoystickButtons[ElevatorConstants.reverseElevatorButtonNumber].onTrue(
        Commands.run(gripperWrist::runUntilReverseLimit, gripperWrist));
    // Commands.run(gripperWrist::runToElevatorSamePos, gripperWrist));
    // actionJoystickButtons[ElevatorConstants.reverseElevatorButtonNumber].onTrue(Commands.run(gripperWrist::runToElevatorOppositePos, gripperWrist));
    actionJoystickButtons[ElevatorConstants.reverseElevatorButtonNumber].onFalse(
        Commands.runOnce(gripperWrist::stop, gripperWrist));
    actionJoystickButtons[ElevatorConstants.reverseElevatorButtonNumber].onTrue(
        Commands.runOnce(pneumatics::tripUp, pneumatics));

    actionJoystickButtons[ElevatorConstants.toTravelElevatorButtonNumber].onTrue(
        Commands.run(elevator::runToTravelPosition, elevator));
    actionJoystickButtons[ElevatorConstants.toTravelElevatorButtonNumber].onFalse(
        Commands.runOnce(elevator::stop, elevator));
    actionJoystickButtons[ElevatorConstants.toTravelElevatorButtonNumber].onTrue(
        Commands.run(elevatorCarriage::runToTravelPosition, elevatorCarriage));
    actionJoystickButtons[ElevatorConstants.toTravelElevatorButtonNumber].onFalse(
        Commands.runOnce(elevatorCarriage::stopCarriage, elevatorCarriage));
    actionJoystickButtons[ElevatorConstants.toTravelElevatorButtonNumber].onTrue(
        Commands.run(gripperWrist::runToElevatorSamePos, gripperWrist));
    // actionJoystickButtons[ElevatorConstants.toTravelElevatorButtonNumber].onTrue(Commands.run(gripperWrist::runToElevatorOppositePos, gripperWrist));
    actionJoystickButtons[ElevatorConstants.toTravelElevatorButtonNumber].onFalse(
        Commands.runOnce(gripperWrist::stop, gripperWrist));
    actionJoystickButtons[ElevatorConstants.toTravelElevatorButtonNumber].onTrue(
        Commands.runOnce(pneumatics::tripUp, pneumatics));

    actionJoystickButtons[ElevatorConstants.toLowScoreElevatorButtonNumber].onTrue(
        Commands.run(elevator::runToLowScorePosition, elevator));
    actionJoystickButtons[ElevatorConstants.toLowScoreElevatorButtonNumber].onFalse(
        Commands.runOnce(elevator::stop, elevator));
    actionJoystickButtons[ElevatorConstants.toLowScoreElevatorButtonNumber].onTrue(
        Commands.run(elevatorCarriage::runToLowScorePosition, elevatorCarriage));
    actionJoystickButtons[ElevatorConstants.toLowScoreElevatorButtonNumber].onFalse(
        Commands.runOnce(elevatorCarriage::stopCarriage, elevatorCarriage));
    actionJoystickButtons[ElevatorConstants.toLowScoreElevatorButtonNumber].onTrue(
        Commands.run(gripperWrist::runToElevatorSamePos, gripperWrist));
    // actionJoystickButtons[ElevatorConstants.toLowScoreElevatorButtonNumber].onTrue(Commands.run(gripperWrist::runToElevatorOppositePos, gripperWrist));
    actionJoystickButtons[ElevatorConstants.toLowScoreElevatorButtonNumber].onFalse(
        Commands.runOnce(gripperWrist::stop, gripperWrist));
    actionJoystickButtons[ElevatorConstants.toLowScoreElevatorButtonNumber].onTrue(
        Commands.runOnce(pneumatics::tripUp, pneumatics));

    actionJoystickButtons[ElevatorConstants.toHighScoreElevatorButtonNumber].onTrue(
        Commands.run(elevator::runToHighScorePosition, elevator));
    actionJoystickButtons[ElevatorConstants.toHighScoreElevatorButtonNumber].onFalse(
        Commands.runOnce(elevator::stop, elevator));
    actionJoystickButtons[ElevatorConstants.toHighScoreElevatorButtonNumber].onTrue(
        Commands.run(elevatorCarriage::runToHighScorePosition, elevatorCarriage));
    actionJoystickButtons[ElevatorConstants.toHighScoreElevatorButtonNumber].onFalse(
        Commands.runOnce(elevatorCarriage::stopCarriage, elevatorCarriage));
    actionJoystickButtons[ElevatorConstants.toHighScoreElevatorButtonNumber].onTrue(
        Commands.run(gripperWrist::runToElevatorSamePos, gripperWrist));
    // actionJoystickButtons[ElevatorConstants.toHighScoreElevatorButtonNumber].onTrue(Commands.run(gripperWrist::runToElevatorOppositePos, gripperWrist));
    actionJoystickButtons[ElevatorConstants.toHighScoreElevatorButtonNumber].onFalse(
        Commands.runOnce(gripperWrist::stop, gripperWrist));
    actionJoystickButtons[ElevatorConstants.toHighScoreElevatorButtonNumber].onTrue(
        Commands.runOnce(pneumatics::tripUp, pneumatics));

    actionJoystickButtons[ElevatorConstants.toScoreGroundButtonNumber].onTrue(
        Commands.run(elevator::runToGroundScorePosition, elevator));
    actionJoystickButtons[ElevatorConstants.toScoreGroundButtonNumber].onFalse(
        Commands.runOnce(elevator::stop, elevator));
    actionJoystickButtons[ElevatorConstants.toScoreGroundButtonNumber].onTrue(
        Commands.run(elevatorCarriage::runToGroundScorePosition, elevatorCarriage));
    actionJoystickButtons[ElevatorConstants.toScoreGroundButtonNumber].onFalse(
        Commands.runOnce(elevatorCarriage::stopCarriage, elevatorCarriage));
    actionJoystickButtons[ElevatorConstants.toScoreGroundButtonNumber].onTrue(
        Commands.run(gripperWrist::runToElevatorSamePos, gripperWrist));
    // actionJoystickButtons[ElevatorConstants.toScoreGroundButtonNumber].onTrue(Commands.run(gripperWrist::runToElevatorOppositePos, gripperWrist));
    actionJoystickButtons[ElevatorConstants.toScoreGroundButtonNumber].onFalse(
        Commands.runOnce(gripperWrist::stop, gripperWrist));
    actionJoystickButtons[ElevatorConstants.toScoreGroundButtonNumber].onTrue(
        Commands.runOnce(pneumatics::tripUp, pneumatics));

    actionJoystickButtons[ElevatorConstants.toPickUpHumanButtonNumber].onTrue(
        Commands.run(elevator::runToHumanPickUpPosition, elevator));
    actionJoystickButtons[ElevatorConstants.toPickUpHumanButtonNumber].onFalse(
        Commands.runOnce(elevator::stop, elevator));
    actionJoystickButtons[ElevatorConstants.toPickUpHumanButtonNumber].onTrue(
        Commands.run(elevatorCarriage::runToHumanPickUpPosition, elevatorCarriage));
    actionJoystickButtons[ElevatorConstants.toPickUpHumanButtonNumber].onFalse(
        Commands.runOnce(elevatorCarriage::stopCarriage, elevatorCarriage));
    actionJoystickButtons[ElevatorConstants.toPickUpHumanButtonNumber].onTrue(
        Commands.run(gripperWrist::runToElevatorSamePos, gripperWrist));
    // actionJoystickButtons[ElevatorConstants.toPickUpHumanButtonNumber].onTrue(Commands.run(gripperWrist::runToElevatorOppositePos, gripperWrist));
    actionJoystickButtons[ElevatorConstants.toPickUpHumanButtonNumber].onFalse(
        Commands.runOnce(gripperWrist::stop, gripperWrist));
    actionJoystickButtons[ElevatorConstants.toPickUpHumanButtonNumber].onTrue(
        Commands.runOnce(pneumatics::tripUp, pneumatics));

    /***** ELEVATOR COMMANDS *****/

    /***** LIGHTS COMMANDS *****/

    actionJoystickButtons[LightsConstants.yellowLightButtonNumber].onTrue(
        Commands.runOnce(lights::turnYellowLightsOn, lights));
    actionJoystickButtons[LightsConstants.yellowLightButtonNumber].onFalse(
        Commands.runOnce(lights::turnOffLights, lights));
    actionJoystickButtons[LightsConstants.purpleLightButtonNumber].onTrue(
        Commands.runOnce(lights::turnPurpleLightsOn, lights));
    actionJoystickButtons[LightsConstants.purpleLightButtonNumber].onFalse(
        Commands.runOnce(lights::turnOffLights, lights));

    /*
                actionJoystickButtons[LightsConstants.yellowLightButtonNumber].onTrue(
                    Commands.runOnce(() -> elevator.run(0.2), elevator));
                actionJoystickButtons[LightsConstants.yellowLightButtonNumber].onFalse(
                    Commands.runOnce(elevator::stop, elevator));
                actionJoystickButtons[LightsConstants.purpleLightButtonNumber].onTrue(
                    Commands.runOnce(() -> elevator.run(-0.2), elevator));
                actionJoystickButtons[LightsConstants.purpleLightButtonNumber].onFalse(
                    Commands.runOnce(elevator::stop, elevator));
    */

    /*
            actionJoystickButtons[LightsConstants.yellowLightButtonNumber].onTrue(
                Commands.runOnce(() -> elevator.runCarriage(0.2), elevator));
            actionJoystickButtons[LightsConstants.yellowLightButtonNumber].onFalse(
                Commands.runOnce(elevator::stopCarriage, elevator));
            actionJoystickButtons[LightsConstants.purpleLightButtonNumber].onTrue(
                Commands.runOnce(() -> elevator.runCarriage(-0.2), elevator));
            actionJoystickButtons[LightsConstants.purpleLightButtonNumber].onFalse(
                Commands.runOnce(elevator::stopCarriage, elevator));
    */
    /***** LIGHTS COMMANDS *****/
  }

  /** Use this method to define your commands for autonomous mode. */
  // FIXME: ADD EVENTS FOR AUTONOMOUS PATHS.
  private void configureAutoCommands() {
    // ScoreGamePiece scoreGamePiece =
    // new ScoreGamePiece(drivetrain, elevator, gripperWrist, pneumatics);
    // PickUpGamePiece pickUpGamePiece =
    // new PickUpGamePiece(drivetrain, elevator, gripperWrist, pneumatics);
    // StartingCondition startingCondition =
    // new StartingCondition(drivetrain, elevator, gripperWrist, pneumatics);
    // CenterOnChargingPad centerOnChargingPad =
    // new CenterOnChargingPad(drivetrain, elevator, gripperWrist, pneumatics);
    AUTO_EVENT_MAP.put("event1", Commands.print("passed marker 1"));
    AUTO_EVENT_MAP.put("event2", Commands.print("passed marker 2"));

    // AUTO_EVENT_MAP.put("Start1", startingCondition);
    // AUTO_EVENT_MAP.put("Score Game Piece", scoreGamePiece);
    // AUTO_EVENT_MAP.put("Balance", centerOnChargingPad);

    // AUTO_EVENT_MAP.put("Start2", startingCondition);
    // AUTO_EVENT_MAP.put("Score First Game Piece", scoreGamePiece);
    // AUTO_EVENT_MAP.put("Pick Up Second Game Piece", pickUpGamePiece);
    // AUTO_EVENT_MAP.put("Score Second Game Piece", scoreGamePiece);
    // AUTO_EVENT_MAP.put("Center on Charging Pad", centerOnChargingPad);

    // AUTO_EVENT_MAP.put("StartB", startingCondition);
    // AUTO_EVENT_MAP.put("BalanceB", centerOnChargingPad);

    // AUTO_EVENT_MAP.put("StartS", startingCondition);
    // AUTO_EVENT_MAP.put("ScoreS", scoreGamePiece);

    // build auto path commands
    List<PathPlannerTrajectory> auto1Paths =
        PathPlanner.loadPathGroup(
            "testPaths1",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Command autoTest =
        Commands.sequence(
            new FollowPathWithEvents(
                new FollowPath(auto1Paths.get(0), drivetrain, true),
                auto1Paths.get(0).getMarkers(),
                AUTO_EVENT_MAP),
            Commands.runOnce(drivetrain::enableXstance, drivetrain),
            Commands.waitSeconds(5.0),
            Commands.runOnce(drivetrain::disableXstance, drivetrain),
            new FollowPathWithEvents(
                new FollowPath(auto1Paths.get(1), drivetrain, false),
                auto1Paths.get(1).getMarkers(),
                AUTO_EVENT_MAP));

    List<PathPlannerTrajectory> game1PiecePaths =
        PathPlanner.loadPathGroup(
            "1GamePiece",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Command runGame1PieceCommand =
        Commands.sequence(
            new StartingCondition(drivetrain, elevator, elevatorCarriage, gripperWrist, pneumatics),
            new FollowPathWithEvents(
                new FollowPath(game1PiecePaths.get(0), drivetrain, true),
                game1PiecePaths.get(0).getMarkers(),
                AUTO_EVENT_MAP),
            new ScoreGamePiece(drivetrain, elevator, elevatorCarriage, gripperWrist, pneumatics),
            new FollowPathWithEvents(
                new FollowPath(game1PiecePaths.get(1), drivetrain, true),
                game1PiecePaths.get(1).getMarkers(),
                AUTO_EVENT_MAP),
            new FollowPathWithEvents(
                new FollowPath(game1PiecePaths.get(2), drivetrain, true),
                game1PiecePaths.get(2).getMarkers(),
                AUTO_EVENT_MAP),
            new CenterOnChargingPad(
                drivetrain, elevator, elevatorCarriage, gripperWrist, pneumatics));

    List<PathPlannerTrajectory> game2PiecePaths =
        PathPlanner.loadPathGroup(
            "2GamePiece",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Command runGame2PieceCommand =
        Commands.sequence(
            new StartingCondition(drivetrain, elevator, elevatorCarriage, gripperWrist, pneumatics),
            new FollowPathWithEvents(
                new FollowPath(game2PiecePaths.get(0), drivetrain, true),
                game2PiecePaths.get(0).getMarkers(),
                AUTO_EVENT_MAP),
            new ScoreGamePiece(drivetrain, elevator, elevatorCarriage, gripperWrist, pneumatics),
            new FollowPathWithEvents(
                new FollowPath(game2PiecePaths.get(1), drivetrain, true),
                game2PiecePaths.get(1).getMarkers(),
                AUTO_EVENT_MAP),
            new PickUpGamePiece(drivetrain, elevator, elevatorCarriage, gripperWrist, pneumatics),
            new FollowPathWithEvents(
                new FollowPath(game2PiecePaths.get(2), drivetrain, true),
                game2PiecePaths.get(2).getMarkers(),
                AUTO_EVENT_MAP),
            new FollowPathWithEvents(
                new FollowPath(game2PiecePaths.get(3), drivetrain, true),
                game2PiecePaths.get(3).getMarkers(),
                AUTO_EVENT_MAP),
            new FollowPathWithEvents(
                new FollowPath(game2PiecePaths.get(4), drivetrain, true),
                game2PiecePaths.get(4).getMarkers(),
                AUTO_EVENT_MAP),
            new ScoreGamePiece(drivetrain, elevator, elevatorCarriage, gripperWrist, pneumatics),
            new FollowPathWithEvents(
                new FollowPath(game2PiecePaths.get(5), drivetrain, true),
                game2PiecePaths.get(5).getMarkers(),
                AUTO_EVENT_MAP),
            new CenterOnChargingPad(
                drivetrain, elevator, elevatorCarriage, gripperWrist, pneumatics));

    List<PathPlannerTrajectory> balancePaths =
        PathPlanner.loadPathGroup(
            "BalancePath",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Command runBalancePathCommand =
        Commands.sequence(
            new StartingCondition(drivetrain, elevator, elevatorCarriage, gripperWrist, pneumatics),
            new FollowPathWithEvents(
                new FollowPath(balancePaths.get(0), drivetrain, true),
                balancePaths.get(0).getMarkers(),
                AUTO_EVENT_MAP),
            new FollowPathWithEvents(
                new FollowPath(balancePaths.get(1), drivetrain, true),
                balancePaths.get(1).getMarkers(),
                AUTO_EVENT_MAP),
            new CenterOnChargingPad(
                drivetrain, elevator, elevatorCarriage, gripperWrist, pneumatics));

    List<PathPlannerTrajectory> justScorePaths =
        PathPlanner.loadPathGroup(
            "JustScore",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Command runJustScorePathCommand =
        Commands.sequence(
            new StartingCondition(drivetrain, elevator, elevatorCarriage, gripperWrist, pneumatics),
            new FollowPathWithEvents(
                new FollowPath(justScorePaths.get(0), drivetrain, true),
                justScorePaths.get(0).getMarkers(),
                AUTO_EVENT_MAP),
            new ScoreGamePiece(drivetrain, elevator, elevatorCarriage, gripperWrist, pneumatics),
            new FollowPathWithEvents(
                new FollowPath(justScorePaths.get(1), drivetrain, true),
                justScorePaths.get(1).getMarkers(),
                AUTO_EVENT_MAP),
            new FollowPathWithEvents(
                new FollowPath(justScorePaths.get(2), drivetrain, true),
                justScorePaths.get(2).getMarkers(),
                AUTO_EVENT_MAP));

    List<PathPlannerTrajectory> game1PieceLeftPaths =
        PathPlanner.loadPathGroup(
            "1GamePieceLeft",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Command runGame1PieceLeftCommand =
        Commands.sequence(
            new StartingCondition(drivetrain, elevator, elevatorCarriage, gripperWrist, pneumatics),
            new FollowPathWithEvents(
                new FollowPath(game1PieceLeftPaths.get(0), drivetrain, true),
                game1PieceLeftPaths.get(0).getMarkers(),
                AUTO_EVENT_MAP),
            new ScoreGamePiece(drivetrain, elevator, elevatorCarriage, gripperWrist, pneumatics),
            new FollowPathWithEvents(
                new FollowPath(game1PieceLeftPaths.get(1), drivetrain, true),
                game1PieceLeftPaths.get(1).getMarkers(),
                AUTO_EVENT_MAP),
            new FollowPathWithEvents(
                new FollowPath(game1PieceLeftPaths.get(2), drivetrain, true),
                game1PieceLeftPaths.get(2).getMarkers(),
                AUTO_EVENT_MAP),
            new CenterOnChargingPad(
                drivetrain, elevator, elevatorCarriage, gripperWrist, pneumatics));

    List<PathPlannerTrajectory> easyBalancePaths =
        PathPlanner.loadPathGroup(
            "EasyBalancePath",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Command runEasyBalanceCommand =
        Commands.sequence(
            new StartingCondition(drivetrain, elevator, elevatorCarriage, gripperWrist, pneumatics),
            new FollowPathWithEvents(
                new FollowPath(easyBalancePaths.get(0), drivetrain, true),
                easyBalancePaths.get(0).getMarkers(),
                AUTO_EVENT_MAP),
            new CenterOnChargingPad(
                drivetrain, elevator, elevatorCarriage, gripperWrist, pneumatics));

    // add commands to the auto chooser
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

    // demonstration of PathPlanner path group with event markers
    autoChooser.addOption("Test Path", autoTest);

    // 1 Game Piece Path
    autoChooser.addOption("1 Game Piece", runGame1PieceCommand);

    // 2 Game Piece Path
    autoChooser.addOption("2 Game Piece", runGame2PieceCommand);

    // Balance Path
    autoChooser.addOption("Balance", runBalancePathCommand);

    // Just Score Path
    autoChooser.addOption("Just Score", runJustScorePathCommand);

    // 1 Game Piece Path Left Side of Field
    autoChooser.addOption("1 Game Piece Left", runGame1PieceLeftCommand);

    // Easy Balance Path
    autoChooser.addOption("Easy Balance Path", runEasyBalanceCommand);

    //Run Forward Command
    autoChooser.addOption("Run Forward", new DriveXMeters(drivetrain, 8.0, true));

    //Center on Charging Pad by Distance
    autoChooser.addOption("Old School Balance", Commands.sequence(new DriveXMeters(drivetrain, 15.0, true), new DriveXMeters(drivetrain, 12.0, true), new CenterOnChargingPad(drivetrain, elevator, elevatorCarriage, gripperWrist, pneumatics)));
    
    //Place Game Piece and Center on Charging Pad by Distance
    autoChooser.addOption("Old School Score and Balance", Commands.sequence(new PlaceAuton(drivetrain, elevator, elevatorCarriage, gripperWrist, pneumatics), new DriveXMeters(drivetrain, 15.0, true), new DriveXMeters(drivetrain, 12.0, true), new CenterOnChargingPad(drivetrain, elevator, elevatorCarriage, gripperWrist, pneumatics)));
    
    // "auto" command for tuning the drive velocity PID
    autoChooser.addOption(
        "Drive Velocity Tuning",
        Commands.sequence(
            Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
            Commands.deadline(
                Commands.waitSeconds(5.0),
                Commands.run(() -> drivetrain.drive(1.5, 0.0, 0.0), drivetrain))));

    // "auto" command for characterizing the drivetrain
    autoChooser.addOption(
        "Drive Characterization",
        new FeedForwardCharacterization(
            drivetrain,
            true,
            new FeedForwardCharacterizationData("drive"),
            drivetrain::runCharacterizationVolts,
            drivetrain::getCharacterizationVelocity));

    Shuffleboard.getTab("MAIN").add(autoChooser.getSendableChooser());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // RunOnChargingPad run = new RunOnChargingPad(drivetrain, elevator, elevatorCarriage,
    // gripperWrist, pneumatics);
    // return run;
    return autoChooser.get();
  }
}
