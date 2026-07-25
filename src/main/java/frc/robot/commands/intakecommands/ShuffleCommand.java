//package frc.robot.commands.intakecommands;
//
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.subsystems.IntakeSubsystem;
//import frc.robot.Constants.IntakeConstants;
//
//public class ShuffleCommand extends Command {
//
//    private IntakeSubsystem intake;
//    private boolean isCommandFinished;
//    private STATE currentState;
//
//
//    private double StarboardDistance;
//    private double PortDistance;
//
//    private double innerThreshold = IntakeConstants.INTAKE_RAIL_END * 0.1;
//    private double outerThreshold = IntakeConstants.INTAKE_RAIL_END * 0.6;
//    private double railEnd = IntakeConstants.INTAKE_RAIL_END;
//
//
//    enum STATE {
//        EXTEND,
//        RETRACT,
//        FINISHED
//    }
//
//    public ShuffleCommand(IntakeSubsystem intake){
//        this.isCommandFinished = false;
//        this.intake = intake;
//    }
//
//    @Override
//    public void initialize() {
//        super.initialize();
//    }
//
//    @Override
//    public void execute() {
//        StarboardDistance = intake.getStarboardPosition();
//        PortDistance = intake.getPortPosition();
//
//        if(StarboardDistance <= innerThreshold || PortDistance <= innerThreshold){
//            currentState = STATE.EXTEND;
//        }else if (StarboardDistance <= outerThreshold || PortDistance <= outerThreshold
//                || StarboardDistance <= railEnd || PortDistance <= railEnd){
//            currentState = STATE.RETRACT;
//        }else{
//            currentState = STATE.EXTEND;
//        }
//
//
//        switch(currentState){
//            case EXTEND:
//                intake.variableExtendIntake(0.6);
//
//                break;
//            case RETRACT:
//                intake.variableExtendIntake(-0.6);
//                break;
//
//            case FINISHED:
//                isCommandFinished = true;
//                break;
//
//
//        }
//
//    }
//
//
//    @Override
//    public boolean isFinished(){
//        return isCommandFinished;
//    }
//
//
//    @Override
//    public void end(boolean interrupted) {
//
//    }
//}
