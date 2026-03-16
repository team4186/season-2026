// package frc.robot.commands;
//
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.ClimbSubsystem;
// import frc.robot.Constants;
//
// public class ClimbCommand extends Command {
//     private final ClimbSubsystem climbSubsystem;
//     private boolean isClimbFinished;
//     private int buttonCount;
//
//     public ClimbCommand(ClimbSubsystem climbSubsystem) {
//         this.climbSubsystem = climbSubsystem;
//         this.isClimbFinished = false;
//         this.buttonCount = 0;
//         addRequirements(climbSubsystem);
//     }
//
//     @Override
//     public void initialize() { }
//
//     @Override
//     public void execute() {
//         if (buttonCount == 0) {
//             // Deploys climb (according to chris, about 178 degrees to starting angle)
// //        climbSubsystem.updateClimb();
//             buttonCount+=1;
//         } else if (buttonCount == 1) {
//             // Climb up & hold (uses the liftAngle from constants)
// //        climbSubsystem.updateClimb();
//             if (climbSubsystem.isClimbAtSetpoint()){
//                 climbSubsystem.climbStop();
//                 buttonCount+=1;
//             }
//         } else if (buttonCount == 2) {
//
//         }
//         //
//     }
//
//     @Override
//     public void end(boolean interrupted) { }
//
//     @Override
//     public boolean isFinished(){
//         return isClimbFinished;
//     }
// }