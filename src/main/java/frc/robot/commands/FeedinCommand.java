package frc.robot.commands;

import com.ctre.phoenix6.signals.ConnectedMotorValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

//feed in: conveyyer up and indexer in in ONE!!

public class FeedinCommand extends Command{

    private final IndexerSubsystem i_index;
    private final LauncherSubsystem l_launch;
    private final Constants.Mode modeDesired;


    public FeedinCommand(IndexerSubsystem indexSub, LauncherSubsystem launcherSub, Constants.Mode mode ) {
        i_index = indexSub;
        l_launch = launcherSub;
        modeDesired = mode;

        addRequirements(i_index, l_launch);
    }

    @Override
    public void execute() {

        double indexSpeed = 0.0; // safe default
        double conveyerSpeed = 0.0; // safe default

        double desiredIndexSpeed = SmartDashboard.getNumber(Constants.IndexerConstants.indexerSpeedString, Constants.IndexerConstants.kIndexMotorSpeed);
        double desiredConveyerSpeed = SmartDashboard.getNumber(Constants.LauncherConstants.converyMotorString, Constants.LauncherConstants.ConveyerMotorSpeed);
        if (modeDesired == Constants.Mode.FORWARD) {
            System.out.println("mode: Forward");
            indexSpeed = -desiredIndexSpeed;
            conveyerSpeed = desiredConveyerSpeed;
        } else if (modeDesired == Constants.Mode.BACKWARD) {
            System.out.println("mode: Backward");
            indexSpeed = desiredIndexSpeed;
            conveyerSpeed = -desiredConveyerSpeed;
        } else if (modeDesired == Constants.Mode.OFF) {
            System.out.println("mode: Off");
            indexSpeed = 0.0;
            conveyerSpeed = 0.0;
        } else {
            System.out.println("FeedinCommand: Unknown mode");
        }
        System.out.printf("FeedIn: indexSpeed: %f, conveyerSpeed: %f\n", indexSpeed, conveyerSpeed);
        i_index.setIndexerSpeed(indexSpeed);
        l_launch.setConveyerSpeed(conveyerSpeed);
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}





