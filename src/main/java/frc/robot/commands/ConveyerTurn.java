package frc.robot.commands;

import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class ConveyerTurn extends Command {
    private final LauncherSubsystem l_Launch;
    private final double m_speed;


    public ConveyerTurn(LauncherSubsystem subsystem, double speed) {
        l_Launch = subsystem;
        m_speed = speed;
        addRequirements(l_Launch);
    }

    @Override
    public void execute() {
        l_Launch.setConveyerSpeed(m_speed);
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
