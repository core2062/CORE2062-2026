package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.signals.ConnectedMotorValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class FeedinCommand extends Command{

    private final IndexerSubsystem i_index;
    private final DoubleSupplier conveyerSpeed;
    private final DoubleSupplier indexerSpeed;
    private final DoubleSupplier agitateSpeed;

    public FeedinCommand(IndexerSubsystem indexSub, DoubleSupplier conveyer, DoubleSupplier indexer, DoubleSupplier agitater) {
        i_index = indexSub;
        conveyerSpeed = conveyer;
        indexerSpeed = indexer;
        agitateSpeed = agitater;

        addRequirements(i_index);
    }

    @Override
    public void execute() {

        System.out.printf("FeedIn: indexSpeed: %f, conveyerSpeed: %f\n", indexerSpeed.getAsDouble(), conveyerSpeed.getAsDouble());
        i_index.setIndexerSpeed(indexerSpeed.getAsDouble(), agitateSpeed.getAsDouble());
        i_index.setConveyerSpeed(conveyerSpeed.getAsDouble());
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}





