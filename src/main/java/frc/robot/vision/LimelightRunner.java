package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;
import swervelib.SwerveDrive;

import java.util.Map;

import static edu.wpi.first.units.Units.DegreesPerSecond;


public class LimelightRunner {

    private static final LimelightRunner instance = new LimelightRunner();

    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(
        AprilTagFields.k2026RebuiltWelded);

    public final String limelightTurret = LimelightConstants.LIMELIGHT_TURRET;
    public final String limelightClimb = LimelightConstants.LIMELIGHT_ROBOT;

//    private final DoubleSubscriber tvSub;
//    private final DoubleSubscriber txSub;
//    private final DoubleSubscriber tySub;
//    private final DoublePublisher ledPub;


    private LimelightRunner(){
//        NetworkTable turretTable = NetworkTableInstance.getDefault().getTable(Constants.LimelightConstants.LIMELIGHT_TURRET);
//        NetworkTable robotTable = NetworkTableInstance.getDefault().getTable(Constants.LimelightConstants.LIMELIGHT_ROBOT);
//
//        // TODO: Set Limelight Positions programmatically
//
//        tvSub = turretTable.getDoubleTopic("tv").subscribe(0.0);
//        txSub = turretTable.getDoubleTopic("tx").subscribe(0.0);
//        tySub = turretTable.getDoubleTopic("ty").subscribe(0.0);
//        ledPub = turretTable.getDoubleTopic("ledMode").publish();


    }

//
    public static LimelightRunner getInstance(){ return instance; }


    public void update() {
        SmartDashboard.putBoolean("Has Target Tag?", LimelightHelpers.getTV(LimelightConstants.LIMELIGHT_TURRET));
        SmartDashboard.putNumber("tx", LimelightHelpers.getTX(LimelightConstants.LIMELIGHT_TURRET));
        SmartDashboard.putNumber("ty", LimelightHelpers.getTY(LimelightConstants.LIMELIGHT_TURRET));
        // SmartDashboard.putNumber("", hasTargetClimber());

        double distInInches = getTurretTagDistanceInchesTrig( LimelightHelpers.getTY(LimelightConstants.LIMELIGHT_TURRET) );
        SmartDashboard.putNumber("Turret AprilTag Dist w/Trig Inches", distInInches);
        SmartDashboard.putNumber("Turret AprilTag Dist w/Trig Feet", distInInches / 12.0);
        SmartDashboard.putNumber("Turret AprilTag Dist w/Trig Meters", distInInches * 0.0254);
        SmartDashboard.putNumber("Turret AprilTag Dist w/Helper W.R.T. Camera Feet", getDistanceToTagWithHelperWRTCamera( limelightTurret ) * 3.28084);
        SmartDashboard.putNumber("Turret AprilTag Dist w/Helper W.R.T. Robot Feet", getDistanceToTagWithHelperWRTRobot( limelightTurret ) * 3.28084);

        SmartDashboard.putNumber("Turret-Tag-Id", getAprilTagId( limelightTurret ));

        if(LimelightHelpers.getTV( limelightClimb )){
            LimelightHelpers.setLEDMode_ForceOn( limelightClimb );
        }else{
            LimelightHelpers.setLEDMode_ForceOff( limelightClimb );
        }
    }


    /**
     * Information on how to set camera pose based on turret camera
     * Update the camera pose every loop based on your mechanism's current state
//    double forward = 0.3;  // meters, forward from robot center
//    double side = 0.0;     // meters, left of robot center
//    double up = 0.5;       // meters, up from robot center
//    double roll = 0.0;     // degrees
//    double pitch = -30.0;  // degrees (e.g. camera tilted down as arm moves)
//    double yaw = 0.0;      // degrees
//
//    LimelightHelpers.setCameraPose_RobotSpace("",
//        forward, side, up, roll, pitch, yaw
//    );
//
//
//     Us
//     Seed the internal IMU with your external gyro (call while disabled)
//    LimelightHelpers.SetIMUMode("", 1);
//
//    // Switch to internal IMU with external assist when enabled
//    LimelightHelpers.SetIMUMode("", 4);
//    LimelightHelpers.SetIMUAssistAlpha("", 0.001);  // Adjust correction strength
*/


    /**
     * Updates pose using limelight pose estimation
     */
    public void updatePoseEstimate(SwerveDrive swerveDrive) {
        double robotYaw = swerveDrive.getYaw().getDegrees();
        // TODO: Change to limelight robot when installed on chassis
        LimelightHelpers.SetRobotOrientation( limelightClimb , robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2( limelightClimb );

        SmartDashboard.putBoolean("Available Tag?", (limelightMeasurement.tagCount > 0));
        SmartDashboard.putNumber("Angular Velocity Gyro", Math.abs(swerveDrive.getGyro().getYawAngularVelocity().in(DegreesPerSecond)));

        // 2 rotations per second
        if ( limelightMeasurement.tagCount > 0 &&
            Math.abs(swerveDrive.getGyro().getYawAngularVelocity().in(DegreesPerSecond)) < 720.0) {
            // Add vision measurement, StdDevs larger number is lower confidence (0.01 - 0.05)
            SmartDashboard.putString("Pose Measurement:", limelightMeasurement.pose.toString());
            swerveDrive.addVisionMeasurement(
                limelightMeasurement.pose,
                limelightMeasurement.timestampSeconds,
                VecBuilder.fill(
                    LimelightConstants.LIMELIGHT_X_STD_DEVS,
                    LimelightConstants.LIMELIGHT_Y_STD_DEVS,
                    LimelightConstants.LIMELIGHT_HEADING_STD_DEVS)
            );
        }
    }


    /**
     * Setup Turret Limelight pipelines for offset ghost targeting
     */
//    public void turretPipelineSetup( boolean isRedAlliance ) {
//        switchToPipeline(limelightTurret,0);
//        resetFiducial3DOffset(limelightTurret);
//
//        switchToPipeline(limelightTurret, 1);
//        setFiducial3DOffset(limelightTurret, 1.0,1.0,1.000);
//
//        switchToPipeline(limelightTurret, 0);
//        setFiducial3DOffset(limelightTurret, 2.0, 2.0,2.0);
//
////        switchToPipeline(limelightTurret, 1);
////        setFiducial3DOffset(
////                limelightTurret,
////                Constants.StructureConstants.TURRET_TARGET_FORWARD_OFFSET,
////                0,
////                0);
////
////        switchToPipeline(limelightTurret, 2);
////        setFiducial3DOffset(
////                limelightTurret,
////                Constants.StructureConstants.TURRET_TARGET_FORWARD_OFFSET,
////                Constants.StructureConstants.TURRET_TARGET_LEFT_SIDE_OFFSET,
////                0);
////
////        switchToPipeline(limelightTurret, 3);
////        setFiducial3DOffset(limelightTurret,
////                Constants.StructureConstants.TURRET_TARGET_FORWARD_OFFSET,
////                Constants.StructureConstants.TURRET_TARGET_RIGHT_SIDE_OFFSET,
////                0);
//
//
//        if ( isRedAlliance ) {
//            fiducialIdFilterOverride(
//                    limelightTurret,
//                    Constants.StructureConstants.RED_FIDUCIAL_TURRET_IDS
//            );
//
////            // Pipeline 1, all center ids
////            switchToPipeline(limelightTurret, 1);
////            fiducialIdFilterOverride(
////                    limelightTurret,
////                    Constants.StructureConstants.OFFSET_GROUP_CENTER_RED
////            );
////
////            // Pipeline 2, all left ids
////            switchToPipeline(limelightTurret, 2);
////            fiducialIdFilterOverride(
////                    limelightTurret,
////                    Constants.StructureConstants.OFFSET_GROUP_LEFT_RED
////            );
////
////            // Pipeline 3, all right ids
////            switchToPipeline(limelightTurret, 3);
////            fiducialIdFilterOverride(
////                    limelightTurret,
////                    Constants.StructureConstants.OFFSET_GROUP_RIGHT_RED
////            );
//
//            // Controls // TODO: Confirm blue and red
//
//        } else {
//            fiducialIdFilterOverride(
//                    limelightTurret,
//                    Constants.StructureConstants.BLUE_FIDUCIAL_TURRET_IDS
//            );
////
////            // Pipeline 1, all center ids
////            switchToPipeline(limelightTurret, 1);
////            fiducialIdFilterOverride(
////                    limelightTurret,
////                    Constants.StructureConstants.OFFSET_GROUP_CENTER_BLUE
////            );
////
////            // Pipeline 2, all left ids
////            switchToPipeline(limelightTurret, 2);
////            fiducialIdFilterOverride(
////                    limelightTurret,
////                    Constants.StructureConstants.OFFSET_GROUP_LEFT_BLUE
////            );
////
////            // Pipeline 3, all right ids
////            switchToPipeline(limelightTurret, 3);
////            fiducialIdFilterOverride(
////                    limelightTurret,
////                    Constants.StructureConstants.OFFSET_GROUP_RIGHT_BLUE
////            );
//        }
//
////        // Set to capture all tags
////        switchToPipeline(limelightTurret, 0);
//    }



    public boolean hasTargetTurret(){
        return LimelightHelpers.getTV(LimelightConstants.LIMELIGHT_TURRET);
    }


    public boolean hasTargetClimber(){
        return LimelightHelpers.getTV(LimelightConstants.LIMELIGHT_TURRET);
    }


    // id - AprilTag ID number
    // txnc - Horizontal offset from camera center (degrees)
    // tync - Vertical offset from camera center (degrees)
    // ta - Target area (0-100% of image)
    // distToCamera - Distance from camera to target (meters)
    // distToRobot - Distance from robot to target (meters)
    // ambiguity - AprilTag pose ambiguity score
    public LimelightHelpers.RawFiducial[] getAprilTagRawFiducial(String limelightName){
        return LimelightHelpers.getRawFiducials(limelightName);
    }


    // Target tag id
    public double getAprilTagId( String limelightName ) {
        return LimelightHelpers.getFiducialID(limelightName);
    }


    /**
     * Function to grab distance to tag from desired camera.
     *
     * @param limelightName name of limelight network table
     * @return Distance in meters to target tag
     */
    // Below is april tag pose in meters relative to the camera
    public double getDistanceToTagWithHelperWRTCamera(String limelightName){
        Pose3d aprilTag = LimelightHelpers.getTargetPose3d_CameraSpace(limelightName);
        return aprilTag.getZ();
    }


    // Below is april tag pose in meters relative to the Robot
    public double getDistanceToTagWithHelperWRTRobot(String limelightName){
        Pose3d aprilTag = LimelightHelpers.getTargetPose3d_RobotSpace(limelightName);
        return aprilTag.getZ();
    }


    // Returns Distance to scoring april tag in inches
    public double getTurretTagDistanceInchesTrig( double tagYOffset ){
        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 21.48; // inches
        double limelightMountAngleDegrees = 25.0;

        // distance from the target to the floor
        double goalHeightInches = 44.25;

        double angleToGoalDegrees = limelightMountAngleDegrees + tagYOffset;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance in inches
        return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    }


    public double getYawTargetPoseCameraSpace(String turretLimelightName) {
        return LimelightHelpers.getTargetPose_CameraSpace(turretLimelightName)[4];
    }


    // Update Camera Pose, can be used to dynamically update pose updates using turret
    public void setCameraPose(String limelightName, double[] cameraPoseOffset){
        if ( cameraPoseOffset.length != 6 ){
            return;
        }

        LimelightHelpers.setCameraPose_RobotSpace(
            limelightName,
            cameraPoseOffset[0],    // Forward offset (meters)
            cameraPoseOffset[1],    // Right offset (meters)
            cameraPoseOffset[2],    // Height offset (meters)
            cameraPoseOffset[3],    // Roll (degrees)
            cameraPoseOffset[4],    // Pitch (degrees)
            cameraPoseOffset[5]     // Yaw (degrees)
        );
    }


    // TODO: Use in RobotContainer or at match start for blue or red side tags
    // Adjust valid tags for a particular limelight camera
    public void fiducialIdFilterOverride(String limelightName, int[] validTags){
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, validTags);
    }


    // Can only have one offset for each pipeline, 10 total pipelines available, can assign offset each tag and use that as a target
    // Forward, Right, Height
    public void setFiducial3DOffset(String limelightName, double xOffset, double yOffset, double zOffset){
        LimelightHelpers.setFiducial3DOffset(limelightName, xOffset, yOffset, zOffset);
    }


    public void switchToPipeline(String limelightName, int pipelineNum) {
        LimelightHelpers.setPipelineIndex(limelightName, pipelineNum);
    }


    public void resetFiducial3DOffset(String limelightName){
        LimelightHelpers.setFiducial3DOffset(limelightName, 0.0, 0.0, 0.0);
    }


    /**
     * For Limelight turret camera to grab distance info and offset relative to tag in view
     *
     * @return double array: { Response (-1 Fail, 0 Tag Found, 1 Success), xOffset to target, distance in feet }
     */
    public double[] getTurretTagBasicInfo(){
        // Default values
        double status = -1.0;
        double dist = 0.0;
        double txOffset = 0.0;

        // is tag present and get tag id
        if ( hasTargetTurret() ) {

            status = 1.0;
            dist = getDistanceToTagWithHelperWRTCamera( limelightTurret ) * 3.28084;
                    //getTurretTagDistanceInchesTrig( LimelightHelpers.getTY( limelightTurret ) ) / 12.0;
            txOffset = LimelightHelpers.getTX( limelightTurret );
        }

        SmartDashboard.putNumber("Turret_Pipeline_Status", status);

        return new double[]{ status, txOffset, dist };
    }


    /**
     * For Limelight turret camera targeting retrieves available tag, switches to pipeline, takes measurement,
     * then resets pipeline back to general pipeline
     *
     * @return double array: { Response (-1 Fail, 0 Tag Found, 1 Success), xOffset to target, distance in feet }
     */
    public double[] getTurretTagInfoWithOffsetPipeline(){
        // Default values
        double status = -1.0;
        double dist = 0.0;
        double txOffset = 0.0;
        int tagId = (int) getAprilTagId(limelightTurret);

        // is tag present and get tag id
        if ( hasTargetTurret() ) {
            int pipelineId = Constants.StructureConstants.TURRET_FIDUCIAL_PIPELINE.get(tagId);

            // swap to correct pipeline
            switchToPipeline(limelightTurret, pipelineId);
            status = 0.0;

            // Double check target Tag is available in new pipeline
            if (hasTargetTurret()) {
                status = 1.0;
                dist = getDistanceToTagWithHelperWRTCamera( limelightTurret ) * 3.28084;
                        // getTurretTagDistanceInchesTrig( LimelightHelpers.getTY( limelightTurret ) ) / 12.0;
                txOffset = LimelightHelpers.getTX( limelightTurret );
            }
        }

        // Switch back to default pipeline
        switchToPipeline(limelightTurret, 0);

        SmartDashboard.putNumber("Turret_Pipeline_Status", status);

        return new double[]{ status, txOffset, dist };
    }

    // Close function required for Subscribers/Publishers
    public void close(){}
}
