package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LimelightConstants;
import swervelib.SwerveDrive;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static java.lang.Math.tan;


public class LimelightRunner {

    private static final LimelightRunner instance = new LimelightRunner();

    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(
        AprilTagFields.k2026RebuiltWelded);

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

        double distInInches = getTurretDistanceToTagTrig();
        SmartDashboard.putNumber("Turret AprilTag Distance w/Trig Inches", distInInches);
        SmartDashboard.putNumber("Turret AprilTag Distance w/Trig Meters", distInInches * 0.0254);
        SmartDashboard.putNumber("Turret AprilTag Distance w/Helper Meters", getDistanceToTagWithHelper(LimelightConstants.LIMELIGHT_TURRET));

        if(LimelightHelpers.getTV(LimelightConstants.LIMELIGHT_ROBOT)){
            LimelightHelpers.setLEDMode_ForceOn(LimelightConstants.LIMELIGHT_ROBOT);
        }else{
            LimelightHelpers.setLEDMode_ForceOff(LimelightConstants.LIMELIGHT_ROBOT);
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
        LimelightHelpers.SetRobotOrientation(LimelightConstants.LIMELIGHT_ROBOT, robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.LIMELIGHT_ROBOT);

        SmartDashboard.putBoolean("Available Tag?", (limelightMeasurement.tagCount > 0));
        SmartDashboard.putNumber("Angular Velocity Gyro", Math.abs(swerveDrive.getGyro().getYawAngularVelocity().in(DegreesPerSecond)));

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


    public boolean hasTargetTurret(){
        return LimelightHelpers.getTV(LimelightConstants.LIMELIGHT_TURRET);
    }


    public boolean hasTargetClimber(){
        return LimelightHelpers.getTV(LimelightConstants.LIMELIGHT_TURRET);
    }


    /**
     * Function to grab distance to tag from desired camera.
     *
     * @param limelightName name of limelight network table
     * @return Distance in meters to target tag
     */
    // Below is robot pose in meters relative to the robot
    public double getDistanceToTagWithHelper(String limelightName){
        Pose3d aprilTag = LimelightHelpers.getTargetPose3d_RobotSpace(limelightName);
        return aprilTag.getZ();
    }


    // Returns Distance to scoring april tag in inches
    public double getTurretDistanceToTagTrig(){
        //44.25 is height in inches of AprilTag off floor
        //25.0 degrees is mounting angle of limelight
        double tagyOffset = LimelightHelpers.getTY(LimelightConstants.LIMELIGHT_TURRET);
        double angleInRadians = Math.toRadians((25.0 + tagyOffset));
        double distance = 44.25 / tan(angleInRadians);

        return (hasTargetTurret()) ? distance : -1.0;
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
            cameraPoseOffset[1],    // Side offset (meters)
            cameraPoseOffset[2],    // Height offset (meters)
            cameraPoseOffset[3],    // Roll (degrees)
            cameraPoseOffset[4],   // Pitch (degrees)
            cameraPoseOffset[5]     // Yaw (degrees)
        );
    }


    // TODO: Use in RobotContainer or at match start for blue or red side tags
    // Adjust valid tags for a particular limelight camera
    public void fiducialIdFilterOverride(String limelightName, int[] validTags){
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, validTags);
    }


    // Can only have one offset for each pipeline, 10 total pipelines available, can assign offset each tag and use that as a target
    public void setFiducial3DOffset(String limelightName, double xOffset, double yOffset, double zOffset){
        LimelightHelpers.setFiducial3DOffset(limelightName, xOffset, yOffset, zOffset);
    }


    public void close(){}
}
