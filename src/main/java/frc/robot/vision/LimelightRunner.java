package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LimelightConstants;
import swervelib.SwerveDrive;

import static edu.wpi.first.units.Units.DegreesPerSecond;


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
//        boolean tagInView = hasTargetTag();
//        SmartDashboard.putBoolean("Has Target Tag?", tagInView);
//        ledPub.set( tagInView? 3.0 : 1.0 );

        SmartDashboard.putBoolean("Has Target Tag?", LimelightHelpers.getTV(LimelightConstants.LIMELIGHT_TURRET));
        SmartDashboard.putNumber("tx", LimelightHelpers.getTX(LimelightConstants.LIMELIGHT_TURRET));
        SmartDashboard.putNumber("ty", LimelightHelpers.getTY(LimelightConstants.LIMELIGHT_TURRET));
        SmartDashboard.putNumber("Distance to turret tag", getDistanceToTag(LimelightConstants.LIMELIGHT_TURRET));

//        SmartDashboard.putNumber("X Offset", tagOffset)
//        SmartDashboard.putNumber("Y Offset", yOffset)
//        SmartDashboard.putNumber("% of Image", tagArea)
//        SmartDashboard.putNumber("Distance", Units.metersToInches(distance))
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

//    private boolean hasTargetTag(){ return tvSub.get() > 0.0; }


    /**
     * Updates pose using limelight pose estimation
     */
    public void updatePoseEstimate(SwerveDrive swerveDrive) {
        double robotYaw = swerveDrive.getYaw().getDegrees();
        // TODO: Change to limelight robot when installed on chassis
        LimelightHelpers.SetRobotOrientation(LimelightConstants.LIMELIGHT_TURRET, robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.LIMELIGHT_TURRET);

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


    /**
     * Function to grab distance to tag from desired camera.
     *
     * @param tablename name of limelight network table
     * @return Distance in meters to target tag
     */
    public double getDistanceToTag(String tablename){
        return LimelightHelpers.getBotPose3d(tablename).getZ();
    }


    public void close(){
//        tvSub.close();
//        txSub.close();
//        tySub.close();
//        ledPub.close();
    }
}
