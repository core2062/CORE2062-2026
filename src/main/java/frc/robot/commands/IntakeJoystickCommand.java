package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;

public class IntakeJoystickCommand extends Command{
    private final IntakeSubsystem i_intake;
    private final DoubleSupplier m_speedSupplier;
    private boolean changedSpeed = false;

    public IntakeJoystickCommand(IntakeSubsystem subsystem, DoubleSupplier setLiftSpeed){
        i_intake = subsystem;
        m_speedSupplier=setLiftSpeed;
        addRequirements(i_intake);
    }

    @Override
    public void execute() {
        double y = -m_speedSupplier.getAsDouble()*Constants.IntakeConstants.kPivotMotorSpeed;
        if (Math.abs(y) > 0.02) {
            i_intake.setPivotSpeed(y);
            changedSpeed = true;
        } else {
            if (changedSpeed) {
                i_intake.setPivotSpeed(0);
                changedSpeed=false;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished(){
     return false;
    }
}