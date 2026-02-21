package frc.robot.commands;

import frc.robot.subsystems.IndexerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class IndexerCommand extends Command {
    private final IndexerSubsystem i_intake;
    private final double m_speed;


    public IndexerCommand(IndexerSubsystem subsystem, double speed) {
        i_intake = subsystem;
        m_speed = speed;
        addRequirements(i_intake);
    }

    @Override
    public void execute() {
        i_intake.setIndexerSpeed(m_speed);
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
