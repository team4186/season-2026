package frc.robot.commands.climbCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.Constants.ClimbConstants;


public class DeployClimbCommand extends Command {
    private final ClimbSubsystem climbSubsystem;

    private boolean isFinished = false;


    public DeployClimbCommand(ClimbSubsystem climbSubsystem){
        this.climbSubsystem = climbSubsystem;
        addRequirements(this.climbSubsystem);

    }


    @Override
    public void initialize(){}


    @Override
    public void execute(){
        if(climbSubsystem.getPosition()<ClimbConstants.CLIMB_THRESHOLD_ANGLE){
            climbSubsystem.simpleClimbDeploy(ClimbConstants.CLIMB_FAST_SPEED);
        }else{
            climbSubsystem.climbStop();
        }
    }


    @Override
    public boolean isFinished(){ return isFinished; }


    @Override
    public void end(boolean interrupted){
        isFinished = false;
        climbSubsystem.climbStop();
    }
}
