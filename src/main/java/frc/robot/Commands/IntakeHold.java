package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;

public class IntakeHold extends Command{
    private final IntakeSubsystem i_intake;
    private final IntSupplier m_iterationCount;
    private int count=0;

    public IntakeHold(IntakeSubsystem subsystem, IntSupplier iterations){
        i_intake = subsystem;
        m_iterationCount=iterations;
        addRequirements(i_intake);
        count = 0;
    }

    @Override
    public void execute() {
        i_intake.setPivotSpeed(0.2);
        count++;
    }

    @Override
    public void end(boolean interrupted) {
        i_intake.setPivotSpeed(0);
    }

    @Override
    public boolean isFinished(){
     return count >= m_iterationCount.getAsInt();  
      }
}