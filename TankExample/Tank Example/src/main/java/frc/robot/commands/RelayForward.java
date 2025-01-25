package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RelaySubsystem;

public class RelayForward extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final RelaySubsystem m_subsystem;

    public RelayForward(RelaySubsystem subsystem) {
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        m_subsystem.relayForward();
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.relayStop();

        super.end(interrupted);
    }
}
