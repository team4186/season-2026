package frc.robot.commands.intakecommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.Timer;

public class ShuffleCommand extends Command {

    private IntakeSubsystem intake;
    private boolean isCommandFinished;
    private STATE currentState;

    private Timer shuffleTimer = new Timer();

    private double StarboardDistance;
    private double PortDistance;

    private double innerThreshold = IntakeConstants.INTAKE_RAIL_END * 0.1;
    private double outerThreshold = IntakeConstants.INTAKE_RAIL_END * 0.6;
    private double railEnd = IntakeConstants.INTAKE_RAIL_END;


    enum STATE {
        EXTEND,
        RETRACT,
        FINISHED
    }

    public ShuffleCommand(IntakeSubsystem intake){
        this.isCommandFinished = false;
        this.intake = intake;
    }

    @Override
    public void initialize() {
        currentState = STATE.EXTEND;
        shuffleTimer.restart();
    }

    @Override
    public void execute() {

        boolean timedOut = shuffleTimer.hasElapsed(1.0);

        switch (currentState) {
            case EXTEND:
                intake.variableExtendIntake(0.6);

                if ((StarboardDistance >= outerThreshold &&
                        PortDistance >= outerThreshold) || timedOut) {

                    currentState = STATE.RETRACT;
                    shuffleTimer.restart();
                }
                break;

            case RETRACT:
                intake.variableExtendIntake(-0.6);

                if ((StarboardDistance <= innerThreshold &&
                        PortDistance <= innerThreshold) || timedOut) {

                    currentState = STATE.EXTEND;
                    shuffleTimer.restart();
                }
                break;
        }

    }


    @Override
    public boolean isFinished(){
        return isCommandFinished;
    }


    @Override
    public void end(boolean interrupted) {

    }
}
