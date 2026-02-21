package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List; 

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonVisionSubsystem extends SubsystemBase{

    private final PhotonCamera camera = new PhotonCamera("regArducam2062");
    public PhotonCamera getCamera(){
        return camera;
    }

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

}