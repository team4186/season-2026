package frc.robot.commands.intakecommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

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
    @Override
    public void execute(){
        if(buttonCounts%2 == 0){
            intakeSubsystem.pickupBallsSlow();
        }else if(buttonCounts%2 == 1){
            isFinished = true;
        }
        
    }

    public void button_detect() {
        buttonCounts++;
    }


    public boolean isFinished(){return isFinished;}

    public void end(boolean interrupted){
        isFinished = false;
        intakeSubsystem.stopPickup();
        buttonCounts = 0;
    }
}