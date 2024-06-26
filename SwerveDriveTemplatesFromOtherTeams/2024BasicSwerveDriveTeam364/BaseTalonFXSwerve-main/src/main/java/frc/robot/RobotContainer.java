package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick steering = new Joystick(1);

    /* Driver Buttons */
    private final JoystickButton turtleButton = new JoystickButton(driver, 3);
    private final JoystickButton turboButton = new JoystickButton(driver, 1);
    private final JoystickButton xStanceButton = new JoystickButton(driver, 4);
    private final JoystickButton zeroGyro = new JoystickButton(driver, 9);
    private final JoystickButton robotCentric = new JoystickButton(driver, 5);
    private final JoystickButton runIntakeButton = new JoystickButton(driver, 6);
    private final JoystickButton runOutakeButton = new JoystickButton(driver, 7);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Intake s_Intake = new Intake();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getY() * 0.75, 
                () -> -driver.getX() * 0.75, 
                () -> -steering.getX() * 0.75, 
                () -> robotCentric.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        turtleButton.whileTrue(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getY() * 0.40, 
                () -> -driver.getX() * 0.40, 
                () -> -steering.getX() * 0.40, 
                () -> robotCentric.getAsBoolean()
            )
        );

        turboButton.whileTrue(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getY() * 0.90, 
                () -> -driver.getX() * 0.90, 
                () -> -steering.getX() * 0.90, 
                () -> robotCentric.getAsBoolean()
            )
        );

        xStanceButton.onTrue(Commands.runOnce(() -> s_Swerve.setXStance()));
        zeroGyro.onTrue(Commands.runOnce(() -> s_Swerve.zeroHeading()));
        runIntakeButton.onTrue(Commands.run(() -> s_Intake.runIntake(-((driver.getZ() * -1 + 1) * 0.5))));
        runIntakeButton.onFalse(Commands.runOnce(() -> s_Intake.stopIntake()));
        runOutakeButton.onTrue(Commands.run(() -> s_Intake.runIntake((driver.getZ() * -1 + 1) * 0.5)));
        runOutakeButton.onFalse(Commands.runOnce(() -> s_Intake.stopIntake()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
