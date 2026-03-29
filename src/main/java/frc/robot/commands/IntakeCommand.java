package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;


public class IntakeCommand extends Command {
        private final IntakeSubsystem i_intake;
        private final DoubleSupplier upperSpeed;


    public IntakeCommand(IntakeSubsystem subsystem, DoubleSupplier upper) {
        i_intake = subsystem;
        upperSpeed = upper;
        addRequirements(i_intake);
    }

    @Override
    public void execute() {
        i_intake.setIntakeSpeed(upperSpeed.getAsDouble());
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
