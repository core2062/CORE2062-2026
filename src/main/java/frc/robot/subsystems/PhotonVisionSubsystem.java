package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import org.photonvision.PhotonCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class PhotonVisionSubsystem extends SubsystemBase{

    //Constants
    private final PhotonCamera camera = new PhotonCamera("regArducam2062");  
    private final PIDController anglePID=new PIDController(0.9, 0, 0);
    private final PIDController drivePID=new PIDController(0.4,0,0);
    private final SlewRateLimiter fowardlimit=new SlewRateLimiter(6.0);
    private final SlewRateLimiter rotationlimit=new SlewRateLimiter(12.0);
    private final double targetDistance = 3.9624; // in meters

    //Non constant variables
    private double trenchTurnAngle = 0.0;
    private double trenchRotation;
    private double limitedTrenchTurn = 0.0;
    private double turnAngle = 0.0;
    private double poseAmbiguity = 0.0;
    private double limitedForward = 0.0;
    private double limitedTurn = 0.0;
    private double distanceToHubXY = 0.0;
    private double hubX = 0.0;
    private double hubY = 0.0;
    private double hubZ = 0.0;
    private double turnAngleToTag = 0.0;
    private boolean targetVisible = false;
    private boolean trenchTargetVisible = false;
    private double forwardOutput = 0.0;
    private double rotationOutput = 0.0;
    private boolean finished = false;
    private boolean trenchFinished = false;
    private double trenchX = 0.0;
    private double trenchY = 0.0;


    //Translations
    private final Transform3d tagToHub=new Transform3d(
        new Translation3d(-0.6096, 0.0, 0.3048),
        new Rotation3d()
    );
    private final Transform3d shooterToCamera = new Transform3d(
        new Translation3d(0.0, 0.0, 0.0),
        new Rotation3d(0, 0, Units.degreesToRadians(0))
    );
    public PhotonCamera getCamera(){
        return camera;
    }

    private boolean isValidHubId(int id) {
        return (id == 10 || id == 5 || id == 2 || id == 26 || id == 18 || id == 21);
    }
    private boolean isValidTrenchId(int id) {
        return (id == 1 || id == 6 || id == 17 || id == 22 );
    }

    public PhotonVisionSubsystem(){
        anglePID.enableContinuousInput(-Math.PI, Math.PI);
        anglePID.setTolerance(Units.degreesToRadians(1.5));
        drivePID.setTolerance(0.04);
    }
    private double round(double value, int places) {
    double scale = Math.pow(10, places);
    return Math.round(value * scale) / scale;
}

    @Override 
    public void periodic(){
        targetVisible = false;
        trenchTargetVisible = false;
        finished=false;
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                for (var target : result.getTargets()) {
                    int id=target.getFiducialId();
                    poseAmbiguity = target.getPoseAmbiguity();
                    if (isValidHubId(id)&&poseAmbiguity<0.4) {
                        targetVisible = true;

                        //Coordinate translations 
                        Transform3d cameraToTarget = target.getBestCameraToTarget();
                        Pose3d robotPose = new Pose3d();
                        Pose3d hubPose = robotPose
                          .transformBy(shooterToCamera)
                          .transformBy(cameraToTarget)
                          .transformBy(tagToHub);
                        

                        //Gets hub coordinates
                        hubX=hubPose.getX();
                        hubY=hubPose.getY();
                        hubZ=hubPose.getZ();

                        //Pythagorean theorem
                        distanceToHubXY=Math.sqrt(Math.pow(hubX, 2)+Math.pow(hubY, 2));
                        turnAngle=Math.atan2(hubY, hubX);
                        turnAngleToTag=Math.atan2(cameraToTarget.getY(), cameraToTarget.getX());
                        
                        //PID calculations
                        forwardOutput=drivePID.calculate(distanceToHubXY, targetDistance);
                        rotationOutput=anglePID.calculate(turnAngle,0);

                        //Clamping max values
                        rotationOutput = MathUtil.clamp(rotationOutput,-1,1)*Constants.Swerve.maxAngularVelocity;
                        forwardOutput=MathUtil.clamp(forwardOutput,-1,1)*Constants.Swerve.maxSpeed;

                        //Limiting acceleration
                        limitedTurn=rotationlimit.calculate(rotationOutput);
                        limitedForward=fowardlimit.calculate(forwardOutput);

                        //Checks if within range
                        if (anglePID.atSetpoint()) {
                            limitedTurn = 0;
                        }
                        if (drivePID.atSetpoint()) {
                            limitedForward = 0;
                        }
                        if (anglePID.atSetpoint() && drivePID.atSetpoint()){
                            finished = true;
                        }
                        
                        //Debug values
                        SmartDashboard.putNumber(Constants.PhotonVisionConstants.DISTANCE_STRING, round(distanceToHubXY, 3));
                        SmartDashboard.putNumber("Turn to hub", round(Units.radiansToDegrees(turnAngle), 3));
                        SmartDashboard.putNumber("Raw turn to hub", round(Units.radiansToDegrees(turnAngleToTag), 3));
                        SmartDashboard.putNumber("Tag accuracy", round(poseAmbiguity, 5));
                        SmartDashboard.putNumber("Raw movement to hub", round(forwardOutput,3));
                        SmartDashboard.putNumber("Raw rotation to hub", round(Units.radiansToDegrees(rotationOutput),3));
                        SmartDashboard.putNumber("Limited movement to hub", round(limitedForward,3));
                        SmartDashboard.putNumber("Limited rotation to hub", round(Units.radiansToDegrees(limitedTurn),3));
                    }else if (isValidTrenchId(id) & poseAmbiguity < 0.4){
                        trenchTargetVisible = true;
                        Transform3d cameraToTarget = target.getBestCameraToTarget();
                        Pose3d robotPose = new Pose3d();
                        Pose3d trenchPose = robotPose.transformBy(cameraToTarget);
                        trenchX = trenchPose.getX();
                        trenchY = trenchPose.getY();
                        trenchTurnAngle = Math.atan2(trenchY, trenchX);
                        trenchRotation = anglePID.calculate(trenchTurnAngle,0);
                        trenchRotation = MathUtil.clamp(trenchRotation, -1, 1) * Constants.Swerve.maxAngularVelocity;
                        limitedTrenchTurn=rotationlimit.calculate(trenchRotation);

                        if (anglePID.atSetpoint()) {
                            limitedTrenchTurn = 0.0;
                            trenchFinished = true;
                        }
                    } else{
                        turnAngle=0;
                    }
                }
            }
        }
            
    }

    //Getters
    public boolean atSetpoint(){
        return finished;
    }
    public boolean atTrenchSetpoint(){
        return trenchFinished;
    }
    public boolean hasHubTarget() { 
        return targetVisible; 
    }
    public boolean hasTrenchTarget() { 
        return trenchTargetVisible; 
    }
    public double getDistanceToHub() {
         return distanceToHubXY; 
    }
    public double getTrenchRotation(){
        return limitedTrenchTurn;
    }
    public double getSpeedToHub(){
        return limitedForward;
    }
    public double getRotationToHub(){
        return limitedTurn;
    }
    public double getRawSpeedToHub(){
        return forwardOutput;
    }
    public double getRawRotationToHub(){
        return rotationOutput;
    }
    public double getAngleToHub() { 
        return turnAngle; 
    }

}