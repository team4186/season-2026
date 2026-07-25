package frc.robot.commands.auto;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class DriveBackAndShoot  extends Command {


    private final TurretSubsystem turret;
    private final SwerveSubsystem swerve;
    private final SpindexerSubsystem spindexer;
    private boolean isCommandFinished;

    private STATE currentState;

    private final Timer autoTimer;

    private final Translation2d speed = new Translation2d(-1.0,0.0);
    // Robot State

    enum STATE {
        DRIVE,
        PREPARE,
        SHOOT,
        FINISHED
    }


    public DriveBackAndShoot(TurretSubsystem turret, IntakeSubsystem intake, SwerveSubsystem swerve, SpindexerSubsystem spindexer){
        this.isCommandFinished = false;
        this.turret = turret;
        this.swerve = swerve;
        this.spindexer = spindexer;
        this.autoTimer = new Timer();

        addRequirements(turret,intake,swerve,spindexer);
    }


    @Override
    public void initialize(){
        currentState = STATE.DRIVE;
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

        switch ( currentState ) {
            case DRIVE:
                // Drive to location
                swerve.drive(speed,0.0,false);

                if(autoTimer.get() >= 1.0){
                    autoTimer.reset();
                    currentState = STATE.PREPARE;
                }

                break;
            case PREPARE:
                // Spin up motor, and aim
                turret.updateHoodAngle(5.0);
                turret.updateShooterSpeed(3000);
                swerve.lock();

                if(autoTimer.get() >= 0.1) {
                    autoTimer.reset();
                    currentState = STATE.SHOOT;
                }
                break;

                case SHOOT:
                // Start spindexer, shoot, and shuffle
                spindexer.feed();
                if(autoTimer.get() >= 5.0){
                    currentState = STATE.FINISHED;
                }
                break;
            case FINISHED:
                // Cleanup, set to defaults, set inturrupt
                isCommandFinished = true;
                break;
        }

    }


    @Override
    public boolean isFinished(){
        return isCommandFinished;
    }


    @Override
    public void end(boolean interrupted) {
        swerve.lock();
        spindexer.stopMotors();
        turret.updateHoodAngle(0.0);
        turret.updateTurretRotation(0.0);
        turret.updateShooterSpeed(0.0);
    }
}


