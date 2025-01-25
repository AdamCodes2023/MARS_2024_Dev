package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RelaySubsystem;

public class RelayReverse extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final RelaySubsystem m_subsystem;

    public RelayReverse(RelaySubsystem subsystem) {
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        m_subsystem.relayReverse();
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.relayStop();

        super.end(interrupted);
    }
}
