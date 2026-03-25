package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;


public class IntakeCommand extends Command {
        private final IntakeSubsystem i_intake;
        private final double upperSpeed;
        private final double lowerSpeed;
        private DoubleSupplier m_angleSupplier = null;
        private boolean m_isAngleMode = false;
        private double angle;

    public IntakeCommand(IntakeSubsystem subsystem, double upper) {
        i_intake = subsystem;
        upperSpeed = upper;
        lowerSpeed = lower;
        m_isAngleMode = false;
        addRequirements(i_intake);
    }
    public IntakeCommand(IntakeSubsystem subsystem, DoubleSupplier angle) {
            i_intake = subsystem;
            m_angleSupplier = angle;
            m_isAngleMode = true;
            addRequirements(i_intake);
            upperSpeed=0;
            lowerSpeed=0;
        }

    @Override
    public void initialize() {
        angle = m_angleSupplier.getAsDouble();   // <-- reads fresh value
        System.out.printf("IntakeMovement: Intiailize setting angle to: %f\n", angle);
        i_intake.setIntakeAngle(angle);
    }
    @Override
    public void execute() {
        i_intake.setIntakeSpeed(upperSpeed);
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
