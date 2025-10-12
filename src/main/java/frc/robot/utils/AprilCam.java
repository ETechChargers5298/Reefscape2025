package frc.robot.utils;


import frc.robot.Constants.VisionConstants;
import frc.robot.FieldConstants;
import frc.robot.Robot;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;


/** Add your docs here. */
public class AprilCam {

    private PhotonCamera camera;
    private Transform3d camOffset;
    private PhotonPoseEstimator photonPoseEstimator;
    private List<PhotonPipelineResult> results;
    private PhotonPipelineResult lastResult;

    private PhotonTrackedTarget desiredTarget;
    private List<PhotonTrackedTarget> targets;
    public int closestId;

    // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;
    private Matrix<N3, N1> currentSDs;
    
    
    // Constructor 1
    public AprilCam(String name, Translation3d pos, Rotation3d angle){
        this.camera = new PhotonCamera(name);
        this.camOffset = new Transform3d(pos, angle);
        this.photonPoseEstimator = new PhotonPoseEstimator(FieldConstants.aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camOffset);
     }

     // Constructor 2: simple version
    public AprilCam(String name) {
        this(name,new Translation3d(), new Rotation3d());
    }


    public void update() {
        this.results = camera.getAllUnreadResults();

        // SmartDashboard.putNumber("X", getX());
        // SmartDashboard.putNumber("Y", getY());
        // SmartDashboard.putNumber("Z", getZ());
        // for (int i = 0; i < getTargets().size(); i++) {
        //   SmartDashboard.putString("Id" + i, getTargets().get(i).toString());
        // }
    }

    // --------------------- GETTING TARGETS -------------------)------------ //

    // Checks if the latest result has any targets
    // public boolean hasTarget() {
    //     return result.hasTargets();
    // }

    //Gets all the AprilTag targets the camera can currently see
    public List<PhotonTrackedTarget> getTargets(){
        return targets;
    }

    public void updateClosestVisibleId(List<PhotonTrackedTarget> help) {
        int closestId = -1;
        double closestDistance = 100;

        for(PhotonTrackedTarget t: help) {
            double currentDistance = Math.sqrt(Math.pow(getTargetTransform(t).getX(), 2) + Math.pow(getTargetTransform(t).getY(), 2));
            
            if(currentDistance < closestDistance) {
                closestDistance = currentDistance;
                closestId = t.fiducialId;
            }
        }
        this.closestId = closestId;
    }

    // Gets the current "best" target
    // public PhotonTrackedTarget getBestTarget(){
    //     return result.getBestTarget();
    // }

    // Gets a target object for a specific AprilTag
    public PhotonTrackedTarget getDesiredTarget(int desiredTargetId) {

        //look at each target in the arraylist of targets
        if(getTargets() != null){
            for (PhotonTrackedTarget t: getTargets())
            {
                //look for the target with the desired Id
                if (t.getFiducialId() == desiredTargetId)
                {
                    return t;
                }
            }
        }
        //return null if you can't find the desiredTarget
        System.out.println("AprilCam cannot find desired target " + desiredTargetId);
        return null;
    }

    // Checks if a desired AprilTag is visible
    // public boolean hasDesiredTarget(int desiredTargetId) {
    //     ///use the getDesiredTarget method to see if it returns null (not correct target) or not
    //     if (getDesiredTarget(desiredTargetId)!= null)
    //     {
    //         return true;
    //     }
    //     return false;
    // }

    // --------------------- GETTING DATA FROM A TARGET ------------------------------- //
    // https://docs.photonvision.org/en/v2025.1.1/docs/programming/photonlib/getting-target-data.html#getting-data-from-a-target
    // double yaw = target.getYaw();
    // double pitch = target.getPitch();
    // double area = target.getArea();
    // double skew = target.getSkew(); //not available for AprilTags
    // Transform2d pose = target.getCameraToTarget();
    // List<TargetCorner> corners = target.getCorners();
    // int targetId = target.getfiducialId();
    // double poseAmbiguity = target.getPoseAmbiguity();
    // Transform3d bestCameraToTarget = target.getBestCameraToTarget();
    // Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

    // Gets the Transform3d object of a specific AprilTag object
    private Transform3d getTargetTransform(PhotonTrackedTarget target){
        if(target == null) {
            return null;
        }
        return target.getBestCameraToTarget();
    }

    // // Gets the X value of a desired target
    public double getXDesired(PhotonTrackedTarget target){
        if(target == null) { return Float.NaN; }
        return getTargetTransform(target).getX();
    }

    // // Gets the X value of the "Best" target
    // public double getXBest(){
    //     return getXDesired( results.getBestTarget() );
    // }
    
    // // Gets the Y value of a desired target
    // public double getYDesired(PhotonTrackedTarget target){
    //     if(target == null) { return Float.NaN; }
    //     return getTargetTransform(target).getY();
    // }

    // // Gets the Y value of the "Best" target
    // public double getYBest(){
    //     return getYDesired( results.getBestTarget() );
    // }

    // // Gets the Z value of a desired target
    // public double getZDesired(PhotonTrackedTarget target){
    //     if(target == null) { return Float.NaN; }
    //     return getTargetTransform(target).getZ();
    // }

    // // Gets the Z value of the "Best" target
    // public double getZBest(){
    //     return getZDesired( results.getBestTarget() );
    // }


    // --------------------- POSE ESTIMATION ------------------------------- //

    // Check out this page: https://docs.photonvision.org/en/latest/docs/examples/poseest.html
    // Here is the example Vision class from PhotonVision: https://github.com/PhotonVision/photonvision/blob/main/photonlib-java-examples/poseest/src/main/java/frc/robot/Vision.java
 
   
    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     * {@link getEstimationSDs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */

    //https://github.com/PhotonVision/photonvision/blob/main/photonlib-java-examples/poseest/src/main/java/frc/robot/Vision.java 
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstPose) {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        //photonPoseEstimator.setReferencePose(prevEstPose);

        for (var change : this.results) {
            visionEst = photonPoseEstimator.update(change);
            updateEstimationSDs(visionEst, change.getTargets());
            targets = change.getTargets();

            updateClosestVisibleId(targets);

        }
        return visionEst;
    }


    /*
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     * Uses higher weights for reef tags!
     * This should only be used when there are targets visible
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationSDs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {

        // No pose input. Default to single-tag std devs
        if (estimatedPose.isEmpty()) {
            currentSDs = VisionConstants.SINGLE_TAG_SD;
        } 
        
        // Pose present. Start running Heuristic
        else {
            var estimatedSDs = VisionConstants.SINGLE_TAG_SD;
            int numTags = 0;
            double totalDistance = 0;
            double totalWeight = 0;

            // Loop through all the targets to find difference in distance from current pose & how important that tag is
            for (var target : targets) {
                // Pose3d tagPose = FieldConstants.getTagPose(target.getFiducialId());
                // if(tagPose != null){
                var tagPose = photonPoseEstimator.getFieldTags().getTagPose(target.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                totalDistance += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
                totalWeight += FieldConstants.TAG_WEIGHTS[target.getFiducialId() - 1]; 
            }

            // Use the single tag standard deviations if no tags visible
            if (numTags == 0) {
                currentSDs = VisionConstants.SINGLE_TAG_SD;
            } 
            
            // One or more tags visible, run the full heuristic.
            else {

                // Calculate the average distance changes were
                double avgDist = totalDistance /numTags;
                double avgWeight = totalWeight /numTags;

                // Decrease standard deviations if multiple targets are visible
                if (numTags > 1) estimatedSDs = VisionConstants.MULTI_TAG_SD;
                
                // Increase standard deviations based on average distance
                if (numTags == 1 && avgDist > 4){
                    estimatedSDs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                } else {
                    estimatedSDs = estimatedSDs.times(1 + (avgDist * avgDist / 30));
                }

                // Weight reef tags higher
                estimatedSDs = estimatedSDs.times(avgWeight);
                currentSDs = estimatedSDs;
            }
        }
    }
     

    /*
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationSDs() {
        return currentSDs;
    }
    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }

    
    //method that gets the Id of the "best" target, generally not used by our robot
    // public String getAllTargets(){

    //     Optional<MultiTargetPNPResult> target = result.getMultiTagResult();

    //     return target.get().fiducialIdsUsed.toString();
    // }

   

}
