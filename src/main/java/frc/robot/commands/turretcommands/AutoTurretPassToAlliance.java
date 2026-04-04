package frc.robot.commands.turretcommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.LimelightRunner;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

import java.sql.Driver;
import java.util.List;
import java.util.function.DoubleSupplier;


public class AutoTurretPassToAlliance extends Command {

    private TurretSubsystem turretSubsystem;
    private Timer lastTagTimestamp;
    private LimelightRunner limelightRunner;
    private Translation2d stationLocation;
    private double homeFieldYaw;
    private final double kp = 0.9;

    public AutoTurretPassToAlliance(TurretSubsystem turretSubsystem)
    {
        this.lastTagTimestamp = new Timer();
        this.turretSubsystem = turretSubsystem;
        this.limelightRunner = LimelightRunner.getInstance();

        boolean isRedAlliance = (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red);
        if (isRedAlliance) {
            this.stationLocation = Constants.StructureConstants.RED_SCORING_LOCATION;
            // homeFieldYaw = 0.0;
        } else {
            this.stationLocation = Constants.StructureConstants.BLUE_SCORING_LOCATION;
            // homeFieldYaw = 0.0;
        }

        addRequirements(this.turretSubsystem);
    }


    @Override
    public void initialize() {
        // reset
        //    turretSubsystem.updateHoodAngle(0.0);
        turretSubsystem.updateTurretRotation(0.0);
        //    turretSubsystem.updateShooterSpeed(0.0);

        // start timer
        lastTagTimestamp.start();
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Adjust based on portion of full adjustment

        double[] targetingInfo = limelightRunner.getTurretTagBasicInfo();
        // double[] targetingInfoWithOffset = limelightRunner.getTurretTagInfoWithOffsetPipeline();
        turretSubsystem.moveHoodUp(15,0.18);

        double status = targetingInfo[0];
        double xOffset = targetingInfo[1];
        double distance = targetingInfo[2];

        // -1 is Failure to find tag, skip adjustment
        SmartDashboard.putNumber("Limelight Tracking STATUS", status);
        if ( status >= 0.0) {
            lastTagTimestamp.restart();
            SmartDashboard.putNumber("Limelight Tracking Tx", xOffset);
            try {
                int adjustedDist = (int) distance;
                double currTurretPosition = turretSubsystem.getTurretPosition();
                double desiredAngle = currTurretPosition + (xOffset * kp);

                SmartDashboard.putNumber("Limelight Tracking Tx_ADJUSTED", desiredAngle );

                // Double[] results = Constants.TurretConstants.TURRET_LOOKUP_TABLE.getOrDefault( adjustedDist , new Double[]{0.0, 0.0});
                turretSubsystem.updateTurretRotation(desiredAngle);
                // turretSubsystem.updateHoodAngle( results[1] ); // TODO: Modify to bang bang control
                // turretSubsystem.updateShooterSpeed( results[0] );

                turretSubsystem.updateShooterSpeed(3000);
            } catch ( NullPointerException e ) {
                SmartDashboard.getNumber("Error_LookupTable", distance);
            }
        }

        // Last target seen > 1 second ago
        SmartDashboard.putBoolean("Limelight Tracking TimeElapsed", lastTagTimestamp.hasElapsed(0.5));
        if (lastTagTimestamp.hasElapsed(0.5)){
            // reset to zero
            //turretSubsystem.updateShooterSpeed(0.0);
            // turretSubsystem.updateHoodAngle(0.0);
            turretSubsystem.updateTurretRotation(0.0);

            // Use Swerve Location and angle
//            double x_robot = SmartDashboard.getNumber("Swerve_X_Position", 0.0);
//            double y_robot = SmartDashboard.getNumber("Swerve_Y_Position", 0.0);
//            double yaw_robot = SmartDashboard.getNumber("Swerve_Yaw_Angle", 0.0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //turretSubsystem.updateShooterSpeed(0.0);
        //turretSubsystem.moveHoodDown(0.0); // TODO: Hood angle is bang bang controller
        turretSubsystem.updateTurretRotation(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }


}
