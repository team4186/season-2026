package frc.robot.commands.climbCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.Constants.ClimbConstants;


public class DeployClimbCommand extends Command {
    private final ClimbSubsystem climbSubsystem;

    private double speed;

    private boolean isFinished = false;


    public DeployClimbCommand(ClimbSubsystem climbSubsystem, double speed){
        this.climbSubsystem = climbSubsystem;
        addRequirements(this.climbSubsystem);
        // Check to make sure speed is et in correct direction
        if (speed < 0) {
            speed = speed * -1;
        }
        this.speed = speed;
    }


    @Override
    public void initialize(){}


    @Override
    public void execute(){
        if(climbSubsystem.getPosition()<ClimbConstants.CLIMB_THRESHOLD_ANGLE){
            climbSubsystem.simpleClimbDeploy(speed);
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
