package frc.robot.commands;

import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class LauncherTurn extends Command{

    private final LauncherSubsystem l_launch;
    private final boolean m_enabled;


public LauncherTurn(LauncherSubsystem subsystem, boolean enabled) {
    l_launch = subsystem;
    m_enabled = enabled;

    addRequirements(subsystem);
}

// Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double upperMotorRps;
    double lowerMotorRps;

    if (m_enabled) {
      upperMotorRps = SmartDashboard.getNumber("desired motor1 RPS", Constants.LauncherConstants.UpperMotorSpeedRpm) / 60.0;
      lowerMotorRps = SmartDashboard.getNumber("desired motor2 RPS", Constants.LauncherConstants.LowerMotorSpeedRpm) / 60.0;
    } else {
     upperMotorRps  = 0.0;
     lowerMotorRps = 0.0;
    }
    l_launch.setShooterSpeed(upperMotorRps, lowerMotorRps);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_enabled;
  }
}




