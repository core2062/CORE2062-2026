package frc.robot.commands;

import frc.robot.subsystems.IndexerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class IndexerCommand extends Command {
    private final IndexerSubsystem i_intake;
    private final double m_indexspeed;
    private final double m_agitatespeed;


    public IndexerCommand(IndexerSubsystem subsystem, double indexSpeed, double agitateSpeed) {
        i_intake = subsystem;
        m_indexspeed = indexSpeed;
        m_agitatespeed=agitateSpeed;
        addRequirements(i_intake);
    }

    @Override
    public void execute() {
        i_intake.setIndexerSpeed(m_indexspeed,m_agitatespeed);
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
