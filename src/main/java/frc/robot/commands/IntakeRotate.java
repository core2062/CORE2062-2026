package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class IntakeRotate extends Command {
        private final IntakeSubsystem i_intake;
        private final int kdegrees;

    public IntakeRotate(IntakeSubsystem subsystem, int degrees) {
        i_intake = subsystem;
        kdegrees = degrees;
        addRequirements(i_intake);
    }

    @Override
    public void execute() {
        i_intake.turnDegrees(kdegrees);
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
