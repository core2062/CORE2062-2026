package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class IntakeCommand extends Command {
        private final IntakeSubsystem i_intake;
        private final double upperSpeed;
        private final double lowerSpeed;


    public IntakeCommand(IntakeSubsystem subsystem, double upper, double lower) {
        i_intake = subsystem;
        upperSpeed = upper;
        lowerSpeed = lower;
        addRequirements(i_intake);
    }

    @Override
    public void execute() {
        i_intake.setIntakeSpeed(upperSpeed, lowerSpeed);
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
