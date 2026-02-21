package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

//feed in: conveyyer up and indexer in in ONE!!

public class FeedinCommand extends Command{

    private final IndexerSubsystem i_index;
    private final LauncherSubsystem l_launch;
    private final double conveyerSpeed;
    private final double indexSpeed;


    public FeedinCommand(IndexerSubsystem indexSub, LauncherSubsystem launcherSub, double conveyer, double index) {
        i_index = indexSub;
        l_launch = launcherSub;
        conveyerSpeed = conveyer;
        indexSpeed = index;

        addRequirements(i_index, l_launch);
    }

    @Override
    public void execute() {
        i_index.setIndexerSpeed(indexSpeed);
        l_launch.setConveyerSpeed(conveyerSpeed);
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}





