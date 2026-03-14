package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;


public class ClimbSubsystem extends SubsystemBase {
    private final SparkMax climbMotor;
    private final DigitalInput homeSwitch;
    private final RelativeEncoder climbEncoder;
    private final SparkClosedLoopController climbMotorController;

    private int endOfTravel;


    public ClimbSubsystem(
            SparkMax climbMotor,
            DigitalInput homeSwitch
    ) {
        this.climbMotor = climbMotor;
        this.homeSwitch = homeSwitch;
        this.climbEncoder = climbMotor.getEncoder();
        this.climbMotorController = climbMotor.getClosedLoopController();
    }


    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Climb_Home_Limit_Switch: ", getLimitSwitch());
        SmartDashboard.putNumber("Climb_Current_Position: ", getPosition());
        //    SmartDashboard.putBoolean("Climb_is_at_Setpoint: ", isClimbAtSetpoint());

        if (getLimitSwitch()) {
            climbEncoder.setPosition(0);
        }
    }


    // Generic updateClimb function, setpoint can be some angle where the arm needs to be deployed.
    // Setpoint could also be some angle where the arm is clamped down.
//    public void updateClimb(double angleSetpoint){
//        climbMotorController.setSetpoint(angleSetpoint, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
//    }

    public void climbStop(){
        climbMotor.stopMotor();
    }

    public double getPosition(){
        return climbEncoder.getPosition();
    }


//    public void resetClimb() {
//        if (!getLimitSwitch()) {
//            climbMotorController.setSetpoint(0, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
//        } else {
//            climbStop();
//            resetEncoder();
//        }
//    }


    public void simpleClimbDeploy(double speed) {
        if (ClimbConstants.CLIMB_DEPLOY_ANGLE >= climbEncoder.getPosition()) {
            climbMotor.set(speed);
        } else {
            climbMotor.stopMotor();
        }
    }


    public void simpleClimbMoveDown(double speed) {
        if (getLimitSwitch()) {
            climbMotor.stopMotor();
        } else {
            climbMotor.set(speed);
        }
    }



//    public boolean isClimbAtSetpoint() {
//        return climbMotorController.isAtSetpoint();
//    }


    public void resetEncoder() {
        climbEncoder.setPosition(0);
    }


    public boolean getLimitSwitch(){
        return !homeSwitch.get();
    }

//commented out for push, uncomment if needed ziyao. - Shing and Rishab
//    public Command deployClimbCommand() {
//        updateClimb(Constants.ClimbConstants.CLIMB_DEPLOY_ANGLE);
//    }
//
//    // Technically climb up and hold
//    public Command climbUpCommand() {
//        // CHANGE CLIMB UP ANGLE, IT IS CURRENTLY SET TO 0.
//        updateClimb(Constants.ClimbConstants.CLIMB_UP_ANGLE);
//             if (isClimbAtSetpoint()) {
//                climbStop();
//             }
//    }
//
//    public Command climbDownCommand() {
//        updateClimb(Constants.ClimbConstants.CLIMB_DOWN_ANGLE);
//        if (isClimbAtSetpoint()) {
//            climbStop();
//        }
//    }
//

}
