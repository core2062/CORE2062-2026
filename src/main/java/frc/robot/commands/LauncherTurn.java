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
    boolean shootertype = SmartDashboard.getBoolean(Constants.LauncherConstants.speedDistance, false);

    if (m_enabled) {

      if (shootertype) {
        System.out.println("shooting based on distance");
        l_launch.distanceShooterSpeed(SmartDashboard.getNumber(Constants.PhotonVisionConstants.DISTANCE_STRING, 3.0));
      } else {
        System.out.println("shooting based on dashboard");
        upperMotorRps = SmartDashboard.getNumber(Constants.LauncherConstants.upperMotorString, l_launch.getUpperTargetRPM()) / 60.0;
        lowerMotorRps = SmartDashboard.getNumber(Constants.LauncherConstants.lowerMotorString, l_launch.getLowerTargetRPM()) / 60.0;
        l_launch.setShooterSpeed(upperMotorRps, lowerMotorRps);
      }

    } else {
     upperMotorRps  = 0.0;
     lowerMotorRps = 0.0;
     l_launch.setShooterSpeed(upperMotorRps, lowerMotorRps);
    }
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_enabled;
  }
}




