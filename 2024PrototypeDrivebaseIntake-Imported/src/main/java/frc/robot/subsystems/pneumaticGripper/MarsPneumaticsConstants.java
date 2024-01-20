package frc.robot.subsystems.pneumaticGripper;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class MarsPneumaticsConstants {
  public static final int pneumaticsModuleID = 5;
  // public static final PneumaticsModuleType pneumaticsModuleType = PneumaticsModuleType.CTREPCM;
  public static final PneumaticsModuleType pneumaticsModuleType = PneumaticsModuleType.REVPH;
  // Define Pneumatics Control Module (PCM) CAN ID

  public static final int singleSolenoid1Channel = 13;
  // Define PCM channel for single solenoid #1

  // public static final int singleSolenoid2Channel = 1;
  // Define PCM channel for single solenoid #2

  public static final int singleSolenoid3Channel = 14;
  // Define PCM channel for single solenoid #3

  public static final int singleSolenoid4Channel = 15;
  // Define PCM channel for single solenoid #4

  public static final int solenoidSingle1SwitchNumber = 10;
  // Define switch number on peumatics button box for single solenoid #1

  // public static final int solenoidSingle2SwitchNumber = 5;
  // Define switch number on peumatics button box for single solenoid #2

  public static final int solenoidSingle3SwitchNumber = 8;
}
