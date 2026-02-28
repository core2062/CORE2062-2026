package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List; 

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonVisionSubsystem extends SubsystemBase{
 private double distanceToHub=0;
    private double distanceError=0;
    private final double distanceAprilTagToHub=0.6096; //0.923798 meters for comp 
    private final double aprilTagHeight=0.7874;
    private double aprilTagDistance=0;
    private double aprilTagRotation=0;
    private double turnAngle=0;
    private double poseAmbiguity=0;
    private double limitedForward=0;
    private double limitedTurn=0;
    private double distanceToHubXY=0;
    private double hubX=0;
    private double hubY=0;
    private double hubZ=0;
    private boolean targetVisible=false;

    
    private final Transform3d tagToHub=new Transform3d(
        new Translation3d(-0.6096, 0.0, 0.3048), //X: -0.6096 Y: 0 Z: 0.3048
        new Rotation3d()
    );
    private final Transform3d robotToCamera = new Transform3d(
    new Translation3d(0.0, 0.0, 0.0), // X, Y, Z in meters
    new Rotation3d(0, 0, Math.PI/2)  // Rotated 90 degrees (left)
);
    private final PhotonCamera camera = new PhotonCamera("regArducam2062");
    public PhotonCamera getCamera(){
        return camera;
    }
    @Override 
    public void periodic(){
         var results = camera.getAllUnreadResults();
                if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    
                    poseAmbiguity = target.getPoseAmbiguity();
                    if(poseAmbiguity<0.5){
                    if (target.getFiducialId() == 1) {
                        targetVisible = true;
                        var transform = target.getBestCameraToTarget();
                        Transform3d cameraToTarget = target.getBestCameraToTarget();
                        Transform3d robotToTarget = robotToCamera.plus(cameraToTarget);
                        Pose3d robotPose = new Pose3d();
                        Pose3d hubPose = robotPose.transformBy(robotToCamera)
                          .transformBy(cameraToTarget)
                          .transformBy(tagToHub);
                         hubX=hubPose.getX();
                         hubY=hubPose.getY();
                         hubZ=hubPose.getZ();
                        aprilTagRotation=transform.getRotation().getZ();
                           if(aprilTagRotation<0){
                                aprilTagRotation+=Math.PI*2;
                            }
                        //aprilTagDistance=Math.sqrt(Math.pow(tagX,2)+Math.pow(tagY, 2)+Math.pow(tagZ, 2));//Distance formula

                        //distanceToHub=Math.sqrt(Math.pow(distanceAprilTagToHub,2)+Math.pow(aprilTagDistance, 2)-2*distanceAprilTagToHub*aprilTagDistance*Math.cos(aprilTagRotation));//Law of cosines
                        //turnAngle=Math.asin(distanceAprilTagToHub*Math.sin(aprilTagRotation)/distanceToHub);//Law sin
                        distanceToHubXY=Math.sqrt(Math.pow(hubX, 2)+Math.pow(hubY, 2));
                        turnAngle=Math.atan2(hubY, hubX)-(Math.PI/2);
                    }
                }
                }
            }
        }
    }
        public boolean hasTarget() { 
        return targetVisible; 
    }
    public double getDistanceToHub() {
         return distanceToHubXY; 
        }
    public double getAngleToHub() { 
        return turnAngle; 
    }
/* 
    public void processVision(){

    PhotonPipelineResult result = camera.getLatestResult();

    //Checks for a target
        if(result.hasTargets()){
        //Finds the best data
        PhotonTrackedTarget target=result.getBestTarget();
        //List of targets
        List<PhotonTrackedTarget> targets = result.getTargets();
        //Gets data from the camera.
        double yaw = target.getYaw();
        double pitch = target.getPitch();
        double area = target.getArea();
        double skew = target.getSkew();
        Transform3d bestCameraToTarget = target.getBestCameraToTarget();
        List<TargetCorner> corners = target.getDetectedCorners();
        Transform3d altPose = target.getAlternateCameraToTarget();
        int targetID = target.getFiducialId();
        double poseAmbiguity = target.getPoseAmbiguity();
        }
    }
*/
}