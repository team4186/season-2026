package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class SimpleSequentialAuto extends Command {

    private TurretSubsystem turret;
    private IntakeSubsystem intake;
    private SwerveSubsystem swerve;
    private boolean isCommandFinished;

    private STATE currStage;

    enum STATE {
        DRIVE,
        PREPARE,
        SHOOT,
        FINISHED
    }


    public SimpleSequentialAuto(){
        this.isCommandFinished = false;
        this.turret = turret;
        this.intake = intake;
        this.swerve = swerve;
    }


    @Override
    public void initialize(){
        currStage = STATE.DRIVE;
    }


    @Override
    public void execute(){

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
