package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.wpilibj.Relay.Value.kOff;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;

public class RelaySubsystem extends SubsystemBase {
    private final Relay m_relay;

    public RelaySubsystem() {
       m_relay = new Relay(0); 
    }

    public void relayForward() {
        m_relay.set(Relay.Value.kForward);
    }

    public void relayReverse() {
        m_relay.set(Relay.Value.kReverse);
    }

    public void relayStop() {
        m_relay.set(kOff);
    }
}
