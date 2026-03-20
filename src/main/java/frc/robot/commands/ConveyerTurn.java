package frc.robot.commands;

import frc.robot.subsystems.IndexerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class ConveyerTurn extends Command {
    private final IndexerSubsystem i_index;
    private final double m_speed;


    public ConveyerTurn(IndexerSubsystem subsystem, double speed) {
        i_index = subsystem;
        m_speed = speed;
        addRequirements(i_index);
    }

    @Override
    public void execute() {
        i_index.setConveyerSpeed(m_speed);
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
