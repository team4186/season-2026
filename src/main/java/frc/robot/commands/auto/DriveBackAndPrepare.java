package frc.robot.commands.auto;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class DriveBackAndPrepare extends Command {

    private final SwerveSubsystem swerve;
    private boolean isCommandFinished;

    private final Timer autoTimer;
    private final double timeout;

    private final Translation2d speed;
    // Robot State

    public DriveBackAndPrepare(SwerveSubsystem swerve, double backupSpeed, double timeout){
        this.isCommandFinished = false;
        this.swerve = swerve;
        this.autoTimer = new Timer();
        this.speed = new Translation2d(backupSpeed,0.0);
        this.timeout = timeout;
        addRequirements(swerve);
    }


    @Override
    public void initialize(){
        swerve.zeroGyroWithAlliance();
        autoTimer.start();
    }


    //FOR REFERENCE:
//            autoChooser.addOption(
//                    "Back Up and Shoot",
//                    Commands.runOnce(drivebase::zeroGyroWithAlliance).withTimeout(.2)
//                        .andThen( turretSubsystem.setShooterMotor(3000).withTimeout(1))
//            .andThen(drivebase.driveBackward().withTimeout(1.0))
//            .andThen(Commands.run(()->turretSubsystem.moveHoodUp(5,0.1)).withTimeout(0.6))
//            .andThen(Commands.run(spindexerSubsystem::feed, spindexerSubsystem).withTimeout(10.0))
//
//            );

    @Override
    public void execute(){
        // Drive to location
        swerve.drive(speed,0.0,false);

        if(autoTimer.get() >= timeout){
            autoTimer.reset();
            swerve.lock();
            isCommandFinished = true;
        }
    }


    @Override
    public boolean isFinished(){
        return isCommandFinished;
    }


    @Override
    public void end(boolean interrupted) {
        swerve.lock();
    }
}


