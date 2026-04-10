package frc.robot.commands.intakecommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;


// Reference: https://docs.wpilib.org/en/stable/docs/software/commandbased/commands.html
public class PickupIntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;

    private boolean isFinished = false;
    private int buttonCounts = 0;


    public PickupIntakeCommand(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(this.intakeSubsystem);
    }


    @Override
    public void initialize(){
    }

    // Shuffle repeatedly
    @Override
    public void execute(){
        // State: at end of length -> move back to shuffle set point

        // State: Shuffle set point to retracted state -> move back to end of length
    }


    public boolean isFinished(){
        return isFinished;
    }


    public void end(boolean interrupted){
        isFinished = false;
        intakeSubsystem.stopPickup();
        buttonCounts = 0;
    }
}