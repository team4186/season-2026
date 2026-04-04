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
    private final double kp = 0.9;
    private ALLIANCE alliance;

    private enum ALLIANCE {
        RED,
        BLUE
    }

    public AutoTurretPassToAlliance(TurretSubsystem turretSubsystem)
    {
        this.turretSubsystem = turretSubsystem;

        boolean isRedAlliance = (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red);
       if (isRedAlliance) {
            this.alliance = ALLIANCE.RED;
        } else {
            this.alliance = ALLIANCE.BLUE;
        }

        addRequirements(this.turretSubsystem);
    }


    @Override
    public void initialize() {
        // reset
        //    turretSubsystem.updateHoodAngle(0.0);
        turretSubsystem.updateTurretRotation(0.0);
        //    turretSubsystem.updateShooterSpeed(0.0);

    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double swerve_yaw = SmartDashboard.getNumber("Swerve_Yaw_Angle", 0.0);

        turretSubsystem.moveHoodUp(35,0.4);
        double xOffset = 0.0;


        if (this.alliance == ALLIANCE.RED) {
            xOffset = Math.min(Math.max( swerve_yaw, 100.0), -100.0);
        }

        if (this.alliance == ALLIANCE.BLUE) {
            // Positive angle = 180 - 100 (goal - maximumTurretRotation)
            if ( swerve_yaw >= 80.0) {
                // Difference * -1 for proper direction
                xOffset = (180 - swerve_yaw) * -1;
            } else if ( swerve_yaw <= -80.0) {
                xOffset = (-180 - swerve_yaw) * -1;
            }
        }

        double desiredAngle = (xOffset * kp);

        SmartDashboard.putNumber("Limelight Tracking Tx_ADJUSTED", desiredAngle );

        // Double[] results = Constants.TurretConstants.TURRET_LOOKUP_TABLE.getOrDefault( adjustedDist , new Double[]{0.0, 0.0});
        turretSubsystem.updateTurretRotation(desiredAngle);
        // turretSubsystem.updateHoodAngle( results[1] ); // TODO: Modify to bang bang control
        // turretSubsystem.updateShooterSpeed( results[0] );

        turretSubsystem.updateShooterSpeed(3000);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        turretSubsystem.updateShooterSpeed(0.0);
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
