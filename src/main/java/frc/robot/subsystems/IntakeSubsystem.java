package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import frc.robot.UnitsUtility;
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {

    private final DigitalInput extendedSwitchStarboard;
    private final DigitalInput extendedSwitchPort;
    private final DigitalInput retractedSwitchStarboard;
    private final DigitalInput retractedSwitchPort;

    private final RelativeEncoder extensionStarboardRelativeEncoder;
    private final RelativeEncoder extensionPortRelativeEncoder;
    private final RelativeEncoder pickupEncoder;

    private final SparkMax extensionStarboardMotor;
    private final SparkMax extensionPortMotor;
    private final SparkMax pickupMotor;

    private final SparkClosedLoopController extensionStarboardController;
    private final SparkClosedLoopController extensionPortController;

    public IntakeSubsystem(
            SparkMax intakeExtensionStarboardMotor,
            SparkMax intakeExtensionPortMotor,
            SparkMax pickupMotor,
//            SparkMax intakeTestMotor,
            DigitalInput extendedSwitch1,
            DigitalInput extendedSwitch2,
            DigitalInput retractedSwitch1,
            DigitalInput retractedSwitch2

    ) {
        this.extensionStarboardMotor = intakeExtensionStarboardMotor;
        this.extensionPortMotor = intakeExtensionPortMotor;
        this.pickupMotor = pickupMotor;

//        this.testMotor = intakeTestMotor;

        this.extendedSwitchStarboard = extendedSwitch1;
        this.extendedSwitchPort = extendedSwitch2;
        this.retractedSwitchStarboard = retractedSwitch1;
        this.retractedSwitchPort = retractedSwitch2;


        this.extensionStarboardRelativeEncoder = intakeExtensionStarboardMotor.getEncoder();
        this.extensionPortRelativeEncoder = intakeExtensionPortMotor.getEncoder();
        this.pickupEncoder = pickupMotor.getEncoder();

        this.extensionStarboardController =  intakeExtensionPortMotor.getClosedLoopController();
        this.extensionPortController = intakeExtensionPortMotor.getClosedLoopController();
    }


    // TODO: What can we publish from this subsystem for system checks and tracking the robot state
    @Override
    public void periodic() {
        //Limit Switch's
        SmartDashboard.putBoolean("Intake_Starboard_Extend_Switch", isStarboardExtended());
        SmartDashboard.putBoolean("Intake_Port_Extend_Switch", isPortExtended());
        SmartDashboard.putBoolean("Intake_Starboard_Retract_Switch", isStarboardRetracted());
        SmartDashboard.putBoolean("Intake_Port_Retract_Switch", isPortRetracted());

        //Encoder Values
        SmartDashboard.putNumber("Intake_Starboard_Position:", getStarboardPosition());
        SmartDashboard.putNumber("Intake_Port_Position:",getPortPosition());
        SmartDashboard.putNumber("Intake_Pickup_Velocity",pickupEncoder.getVelocity());

        SmartDashboard.putBoolean("Intake_Starboard_is_at_setpoint",starboardAtSetpoint());
        SmartDashboard.putBoolean("Intake_Port_is_at_setpoint", portAtSetpoint());

    }

    public boolean starboardAtSetpoint(){
        return extensionStarboardController.isAtSetpoint();
    }

    public boolean portAtSetpoint(){
        return extensionPortController.isAtSetpoint();
    }


    private boolean isStarboardExtended() {
        return !UnitsUtility.isBeamBroken(extendedSwitchStarboard, false, "Intake Extension Switch Starboard");
    }


    private boolean isPortExtended() {
        return !UnitsUtility.isBeamBroken(extendedSwitchPort, false, "Intake Extension Switch Port");
    }


    private boolean isStarboardRetracted(){
        return !UnitsUtility.isBeamBroken(retractedSwitchStarboard, false, "Intake Retracted Switch Starboard");
    }

    private boolean isPortRetracted(){
        return !UnitsUtility.isBeamBroken(retractedSwitchPort, false, "Intake Retracted Switch Port");
    }

    public boolean isIntakeExtended(){
        return isStarboardExtended() && isPortExtended();
    }

    public boolean isIntakeRetracted(){
        return isPortRetracted() && isStarboardRetracted();
    }




    //TODO: MAKE SURE THE MOTORS ARE INVERTED,AND OPPOSITE CORRECTLY!!!!!! VERY IMPORTANT OR STUFF WILL BREAK!!!!! -Shing
    private void extendIntakeStarboard(){
        if(!isStarboardExtended()){
            extensionStarboardController.setSetpoint(IntakeConstants.RAIL_END, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
            // 31.0 is the length of the rail on the intake, in cm. It's the far end of the rail- Shing
        } else {
            extensionStarboardMotor.stopMotor();
        }
    }


    private void extendIntakePort(){
        if(!isPortExtended()){
            extensionPortController.setSetpoint(IntakeConstants.RAIL_END, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
        } else {
            extensionPortMotor.stopMotor();
        }
    }


    // \/\/\/\/\/Is this needed? Not sure how to feed one value into two controllers, to keep both sides in sync. If there's a better way, delete \/\/\/\/\/
//    public void extendIntake() {
////        if(isStarboardExtended() && isPortExtended()){
////            extensionStarboardController.setSetpoint(31.0, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
////            extensionPortController.setSetpoint(31.0, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
////        }else{
////            extensionStarboardMotor.stopMotor();
////            extensionPortMotor.stopMotor();
////        }
//        extendIntakePort();
//        extendIntakeStarboard();
//    }


    private void retractIntakeStarboard(){
        if(!isStarboardRetracted()){
            extensionStarboardController.setSetpoint(IntakeConstants.RAIL_START, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
            // 0 is the other end, the start - Shing
        }else{
            extensionStarboardMotor.stopMotor();
        }
    }

    private void retractIntakePort(){
        if(!isPortRetracted()){
            extensionPortController.setSetpoint(IntakeConstants.RAIL_START, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
            // 0 is the other end, the start - Shing
        }else{
            extensionPortMotor.stopMotor();
        }
    }


    // TODO: intake using CLOSED LOOP
//    public void retractIntake(){
//        retractIntakePort();
//        retractIntakeStarboard();
//    }


    //just feeding a velocity
    public void simplePairExtension(){
        if(!isStarboardExtended()){
            extensionStarboardMotor.set(0.1);
        }
        if(!isPortExtended()){
            extensionPortMotor.set(0.1);
        }
    }


    public void simplePairRetraction(){
        if(!isStarboardRetracted()){
            extensionStarboardMotor.set(-0.1);
        }
        if(!isPortRetracted()){
            extensionPortMotor.set(-0.1);
        }
    }



    public void pickupBallsFast(){
        pickupMotor.set(IntakeConstants.PICKUP_FAST_SPEED);
    }

    public void pickupBallsSlow(){
        pickupMotor.set(IntakeConstants.PICKUP_SLOW_SPEED);
    }

    public void pickupBalls(double speed){
        pickupMotor.set(speed);
    }



    // TODO: Possible Close Loop Implmentation for pickup. Low Priority
    public void updateIntakePickupSpeed() {}
    public void updateIntakePosition() {}

    public void stopTranslation() {
        extensionPortMotor.stopMotor();
        extensionStarboardMotor.stopMotor();    }

    public void stopPickup(){
        pickupMotor.stopMotor();
    }

    public void stop(){
        extensionPortMotor.stopMotor();
        extensionStarboardMotor.stopMotor();
        pickupMotor.stopMotor();
    }



    public double getStarboardPosition(){
        return extensionStarboardRelativeEncoder.getPosition();
    }

    public double getPortPosition(){
        return extensionPortRelativeEncoder.getPosition();
    }

    public Command setSlowPickup(double speed){
        return Commands.runOnce(()->pickupBalls(speed),this).repeatedly();
    }

    public Command stopPickupMotor(){
        return Commands.runOnce(this::stopPickup,this);
    }

}
