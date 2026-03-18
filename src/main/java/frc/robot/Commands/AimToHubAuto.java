package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonVisionSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AimToHubAuto extends Command {

    private CommandSwerveDrivetrain s_Swerve;
    private PhotonVisionSubsystem p_vision;
   private double forward;
   private double rotationOutput;
   private boolean finished = false;
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();

    private double limitedForward=0;
    private double limitedTurn=0;

    public AimToHubAuto(CommandSwerveDrivetrain s_Swerve, PhotonVisionSubsystem p_vision) {
        this.s_Swerve = s_Swerve;
        this.p_vision = p_vision;
        
        // This tells the robot that this command uses the drivetrain
        addRequirements(p_vision);
        addRequirements(s_Swerve);
    }
    @Override
    public void execute(){
           if (p_vision.hasTarget()==true) {
            double turnAngle=0;
            turnAngle=p_vision.getAngleToHub();
            double distanceToHubXY=0;
            distanceToHubXY=p_vision.getDistanceToHub();
            rotationOutput=anglePID.calculate(turnAngle,0)*Constants.Swerve.maxAngularVelocity;
            forward=drivePID.calculate(distanceToHubXY, targetDistance)*Constants.Swerve.maxSpeed;
    if (anglePID.atSetpoint()) {
        rotationOutput = 0;
        System.out.print(distanceToHubXY);
    }
    if (drivePID.atSetpoint()) {
        forward = 0;
    }
            rotationOutput = MathUtil.clamp(rotationOutput,-Constants.Swerve.maxAngularVelocity,Constants.Swerve.maxAngularVelocity);
        forward=MathUtil.clamp(forward,-Constants.Swerve.maxSpeed,Constants.Swerve.maxSpeed);
        limitedTurn=rotationlimit.calculate(rotationOutput);
        limitedForward=fowardlimit.calculate(forward);
}else{
    /*limitedTurn = limitedTurn*0.8; 
    limitedForward = limitedForward*0.8;
    rotationlimit.reset(limitedTurn); 
    fowardlimit.reset(limitedForward);*/
}
//System.out.printf("rotationOutput: %f, limitedTurn: %f, limitedForward %f\n", rotationOutput, limitedTurn, limitedForward);
        if (anglePID.atSetpoint() && drivePID.atSetpoint()){
            finished = true;
        }
        s_Swerve.setControl(driveRequest.withVelocityY(-limitedForward).withRotationalRate(-limitedTurn));
        System.out.printf("AimToHubAuto: finished: %b, limitedForward: %f, limitedTurn: %f\n", finished, limitedForward, -limitedTurn);
        /* \n
        SmartDashboard.putNumber("Rotation of the april tag",aprilTagRotation);
        SmartDashboard.putNumber("Finds distance to april tag", aprilTagDistance);
        SmartDashboard.putNumber("Angle to turn to the hub", Units.radiansToDegrees(rotationOutput));
        SmartDashboard.putNumber("Distance to hub", distanceToHubXY);
        SmartDashboard.putNumber("April Tag Inaccurecy",poseAmbiguity);
        SmartDashboard.putNumber("Forward Photon", limitedForward);
        SmartDashboard.putNumber("Turn photon", -limitedTurn);
        SmartDashboard.putNumber("Finding the rotation of april tag", turnAngle);*/
    }

    @Override
    public boolean isFinished() {
        return finished ;
    }
}