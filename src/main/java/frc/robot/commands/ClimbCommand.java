package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.Constants;

public class ClimbCommand extends Command {
    private final ClimbSubsystem climbSubsystem;
    private boolean isClimbFinished;

    public ClimbCommand(ClimbSubsystem climbSubsystem) {
        this.climbSubsystem = climbSubsystem;
        this.isClimbFinished = false;

        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() { }

    @Override
    public void execute() {
        // Deploys climb
//        climbSubsystem.updateClimb();

        //
    }

    @Override
    public void end(boolean interrupted) { }

    @Override
    public boolean isFinished(){
        return isClimbFinished;
    }
}