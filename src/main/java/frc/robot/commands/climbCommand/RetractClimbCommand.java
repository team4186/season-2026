package frc.robot.commands.climbCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.Constants.ClimbConstants;


public class RetractClimbCommand extends Command {
    private final ClimbSubsystem climbSubsystem;
    private final double speed;

    private boolean isFinished = false;


    public RetractClimbCommand(ClimbSubsystem climbSubsystem, double speed){
        this.climbSubsystem = climbSubsystem;

        // Check to make sure speed is et in correct direction
        if (speed > 0) {
            speed = speed * -1;
        }
        this.speed = speed;

        addRequirements(this.climbSubsystem);
    }


    @Override
    public void initialize(){}


    @Override
    public void execute(){
        if(climbSubsystem.getPosition() >= 0.0){
            climbSubsystem.simpleClimbDeploy(speed); //TODO: move to constants
        }else{
            isFinished = true;
        }
    }


    @Override
    public boolean isFinished(){return isFinished;}


    @Override
    public void end(boolean interrupted){
        isFinished = false;
        climbSubsystem.climbStop();
    }
}
