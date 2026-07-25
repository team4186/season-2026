package frc.robot.commands.auto;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class DriveBackAndShoot  extends Command {

    private TurretSubsystem turret;
    private IntakeSubsystem intake;
    private SwerveSubsystem swerve;
    private boolean isCommandFinished;

    private Pose2d startingCoords;

    private STATE currentState;

    // Robot State


    enum STATE {
        DRIVE,
        PREPARE,
        SHOOT,
        FINISHED
    }


    public DriveBackAndShoot( double distance, double movetimeout, double shooterspeed, double hoodangle ){
        this.isCommandFinished = false;
        this.turret = turret;
        this.intake = intake;
        this.swerve = swerve;
    }


    @Override
    public void initialize(){
        currentState = STATE.DRIVE;
        this.startingCoords = this.swerve.getPose();
    }


    @Override
    public void execute(){

        switch ( currentState ) {
            case DRIVE:
                // Drive to location

                // Check if met conditions for PREPARE start
                /*
                if (conditionsMet) {
                    currentState = STATE.PREPARE;
                }
                 */
                break;
            case PREPARE:
                // Spin up motor, and aim
                break;
            case SHOOT:
                // Start spindexer, shoot, and shuffle
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
        turret.stopHoodMotor();
        turret.updateTurretRotation(0.0);
        turret.updateShooterSpeed(0.0);
    }
}


