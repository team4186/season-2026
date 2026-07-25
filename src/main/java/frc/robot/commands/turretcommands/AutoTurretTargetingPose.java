package frc.robot.commands.turretcommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.vision.LimelightRunner;


public class AutoTurretTargetingPose extends Command {

    private TurretSubsystem turretSubsystem;
    private Timer lastTagTimestamp;
    private LimelightRunner limelightRunner;

    private Translation2d stationLocation;
    private double homeFieldYaw;
    private final double kp = 0.9;



    public AutoTurretTargetingPose(TurretSubsystem turretSubsystem)
    {
        this.lastTagTimestamp = new Timer();
        this.turretSubsystem = turretSubsystem;
        this.limelightRunner = LimelightRunner.getInstance();

        boolean isRedAlliance = (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red);
        if (isRedAlliance) {
            this.stationLocation = Constants.StructureConstants.RED_SCORING_LOCATION;
        } else {
            this.stationLocation = Constants.StructureConstants.BLUE_SCORING_LOCATION;
        }

        addRequirements(this.turretSubsystem);




    }


    @Override
    public void initialize() {
        // reset
        turretSubsystem.updateHoodAngle(0.0);
        turretSubsystem.updateTurretRotation(0.0);
        turretSubsystem.updateShooterSpeed(0.0);

        // start timer
        lastTagTimestamp.start();
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Adjust based on portion of full adjustment

        double[] targetingInfo = limelightRunner.getTurretTagBasicInfo();


        double status = targetingInfo[0];
        double xOffset = targetingInfo[1];

        // -1 is Failure to find tag, skip adjustment
        SmartDashboard.putNumber("Limelight Tracking STATUS", status);
        if ( status >= 0.0) {
            lastTagTimestamp.restart();
            SmartDashboard.putNumber("Limelight Tracking Tx", xOffset);
            try {
                double currTurretPosition = turretSubsystem.getTurretPosition();
                double desiredAngle = currTurretPosition + (xOffset * kp);


                double currentX = SmartDashboard.getNumber("Swerve_X_Position",-67); //This should NEVER EVER not get a number!!!
                double currentY = SmartDashboard.getNumber("Swerve_Y_Position",-67);

                Translation2d robotPose = new Translation2d(currentX,currentY);

                SmartDashboard.putNumber("Limelight Tracking Tx_ADJUSTED", desiredAngle );

                turretSubsystem.updateTurretRotation(desiredAngle);

                turretSubsystem.updateShooterSpeed(LimelightRunner.getInstance().getTurretVelocityUsingPose(robotPose, stationLocation));
                turretSubsystem.updateHoodAngle(LimelightRunner.getInstance().getHoodAngleUsingPose(robotPose, stationLocation));
            } catch ( NullPointerException e ) {
                System.out.println("YOU HAVE AN ERROR! FIX IT! TRIPPLE T COMPLES YOU!");
            }
        }

        // Last target seen > 1 second ago
        SmartDashboard.putBoolean("Limelight Tracking TimeElapsed", lastTagTimestamp.hasElapsed(0.5));
        if (lastTagTimestamp.hasElapsed(1.0)){
            // reset to zero
            turretSubsystem.updateShooterSpeed(0.0);
            turretSubsystem.updateHoodAngle(0.0);
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

        turretSubsystem.updateTurretRotation(0.0);
        turretSubsystem.updateShooterSpeed(0.0);
        turretSubsystem.updateHoodAngle(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }


}
