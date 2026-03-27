package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;


public class IntakeRotate extends Command {
    private final IntakeSubsystem i_intake;
    private double kdegrees;
    private final DoubleSupplier angleSupplier;
    private double targetAngle;
    private boolean run = false;

    public IntakeRotate(IntakeSubsystem subsystem, DoubleSupplier angle) {
        i_intake = subsystem;
        angleSupplier = angle;
        kdegrees = angle.getAsDouble();
        addRequirements(i_intake);
    }

    @Override
    public void initialize() {
        targetAngle = angleSupplier.getAsDouble();   // <-- reads fresh value
        System.out.printf("Intake Rotate: Intiailize setting angle to: %f\n", kdegrees);
        i_intake.turnDegrees(targetAngle);
    }

    @Override
    public void execute() {
        if (!run){
            i_intake.turnDegrees(kdegrees);
            run = true;
        }
    }
    @Override
    public boolean isFinished() {
        return i_intake.isAtAngle(targetAngle);
    }
}
