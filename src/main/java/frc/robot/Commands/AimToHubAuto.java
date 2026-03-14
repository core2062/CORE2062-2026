package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonVisionSubsystem;

import com.ctre.phoenix6.swerve.SwerveRequest;

public class AimToHubAuto extends Command {
    private CommandSwerveDrivetrain s_Swerve;
    private PhotonVisionSubsystem p_vision;
    private GenericHID controller;
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();
    private final PIDController anglePID=new PIDController(0.9, 0, 0);
    private final PIDController drivePID=new PIDController(0.4,0,0);
    private double limitedForward=0;
    private double limitedTurn=0;
      public AimToHubAuto(CommandSwerveDrivetrain s_Swerve, PhotonVisionSubsystem p_vision, GenericHID controller) {
        this.s_Swerve = s_Swerve;
        this.p_vision = p_vision;
        this.controller = controller;
        anglePID.enableContinuousInput(-Math.PI, Math.PI);
        anglePID.setTolerance(Units.degreesToRadians(3));
        drivePID.setTolerance(0.04);
        
        // This tells the robot that this command uses the drivetrain
        addRequirements(s_Swerve,p_vision);
    }
    @Override
    public void execute(){
       
        double forward = -controller.getRawAxis(1) * Constants.Swerve.maxSpeed;
        double rotationOutput = -controller.getRawAxis(4) * Constants.Swerve.maxAngularVelocity;   
           if (p_vision.hasTarget()==true) {
           limitedForward=p_vision.getSpeedToHub();
           limitedTurn=p_vision.getAngleToHub();
        if (anglePID.atSetpoint()) {
            rotationOutput = 0;
        }
        if (drivePID.atSetpoint()) {
            forward = 0;
        }
}else{
    /*limitedTurn = limitedTurn*0.8; 
    limitedForward = limitedForward*0.8;
    rotationlimit.reset(limitedTurn); 
    fowardlimit.reset(limitedForward);*/
}
//System.out.printf("rotationOutput: %f, limitedTurn: %f, limitedForward %f\n", rotationOutput, limitedTurn, limitedForward);
        s_Swerve.setControl(driveRequest.withVelocityY(-limitedForward).withRotationalRate(-limitedTurn));
        
    }
}