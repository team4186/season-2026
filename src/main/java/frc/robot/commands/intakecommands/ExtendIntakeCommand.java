package frc.robot.commands.intakecommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ExtendIntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;

    private boolean isFinished = false;
    private boolean isExtended = false;
    private boolean isRetracted = false;

    public ExtendIntakeCommand(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(this.intakeSubsystem);
    }

    public void initialize(){
    }

    public void execute(){
        if(intakeSubsystem.isIntakeExtended()){
            isFinished = true;
        }else{
            intakeSubsystem.simplePairExtension();
        }

        isExtended = intakeSubsystem.isIntakeExtended();
        isRetracted = intakeSubsystem.isIntakeRetracted();
    }

    public boolean isFinished(){return isFinished;}

    public void end(boolean interrupted){
        isFinished = false;
        intakeSubsystem.stopTranslation();
    }
}