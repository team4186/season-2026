package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import frc.robot.UnitsUtility;


public class IntakeSubsystem extends SubsystemBase {

    private final DigitalInput extendedSwitchStarboard;
    private final DigitalInput extendedSwitchPort;
    private final DigitalInput retractedSwitchStarboard;
    private final DigitalInput retractedSwitchPort;

    private final RelativeEncoder extensionStarboardRelativeEncoder;
    private final RelativeEncoder extensionPortRelativeEncoder;

    private final SparkMax extensionStarboardMotor;
    private final SparkMax extensionPortMotor;
    private final SparkMax pickupMotor;

    private final SparkClosedLoopController extensionStarboardController;
    private final SparkClosedLoopController extensionPortController;

    public IntakeSubsystem(
            SparkMax intakeExtensionStarboardMotor,
            SparkMax intakeExtensionPortMotor,
            SparkMax extendMotor,
            DigitalInput extendedSwitch1,
            DigitalInput extendedSwitch2,
            DigitalInput retractedSwitch1,
            DigitalInput retractedSwitch2

    ) {
        this.extensionStarboardMotor = intakeExtensionStarboardMotor;
        this.extensionPortMotor = intakeExtensionPortMotor;
        this.pickupMotor = extendMotor;

        this.extendedSwitchStarboard = extendedSwitch1;
        this.extendedSwitchPort = extendedSwitch2;
        this.retractedSwitchStarboard = retractedSwitch1;
        this.retractedSwitchPort = retractedSwitch2;


        this.extensionStarboardRelativeEncoder = intakeExtensionStarboardMotor.getEncoder();
        this.extensionPortRelativeEncoder = intakeExtensionPortMotor.getEncoder();

        this.extensionStarboardController =  intakeExtensionPortMotor.getClosedLoopController();
        this.extensionPortController = intakeExtensionPortMotor.getClosedLoopController();
    }


    // TODO: What can we publish from this subsystem for system checks and tracking the robot state
    @Override
    public void periodic() {

    }




    private boolean isStarboardExtended() {
        return UnitsUtility.isBeamBroken(extendedSwitchStarboard, false, "Intake Extension Switch Starboard");
    }


    private boolean isPortExtended() {
        return UnitsUtility.isBeamBroken(extendedSwitchPort, false, "Intake Extension Switch Port");
    }


    private boolean isStarboardRetracted(){
        return UnitsUtility.isBeamBroken(retractedSwitchStarboard, false, "Retracted Extension Switch Starboard");
    }

    private boolean isPortRetracted(){
        return UnitsUtility.isBeamBroken(retractedSwitchPort, false, "Retracted Extension Switch Port");
    }




    //TODO: MAKE SURE THE MOTORS ARE INVERTED,AND OPPOSITE CORRECTLY!!!!!! VERY IMPORTANT OR STUFF WILL BREAK!!!!! -Shing
    private void extendIntakeStarboard(){
        if(!isStarboardExtended()){
            extensionStarboardController.setSetpoint(31.0, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
            // 31.0 is the length of the rail on the intake, in cm. It's the far end of the rail- Shing
        } else {
            extensionStarboardMotor.stopMotor();
        }
    }


    private void extendIntakePort(){
        if(!isPortExtended()){
            extensionPortController.setSetpoint(31.0, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
        } else {
            extensionPortMotor.stopMotor();
        }
    }

    // \/\/\/\/\/Is this needed? Not sure how to feed one value into two controllers, to keep both sides in sync. If there's a better way, delete \/\/\/\/\/
    public void extendIntake() {
        if(isStarboardExtended() && isPortExtended()){
            extensionStarboardController.setSetpoint(31.0, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
            extensionPortController.setSetpoint(31.0, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }else{
            extensionStarboardMotor.stopMotor();
            extensionPortMotor.stopMotor();
        }
    }

    private void retractIntakeStarboard(){
        if(!isStarboardRetracted()){
            extensionStarboardController.setSetpoint(0.0, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
            // 0 is the other end, the start - Shing
        }else{
            extensionStarboardMotor.stopMotor();
        }
    }

    private void retractIntakePort(){
        if(!isPortRetracted()){
            extensionPortController.setSetpoint(0.0, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
            // 0 is the other end, the start - Shing
        }else{
            extensionPortMotor.stopMotor();
        }
    }

    // TODO: Decision -> closed loop kVelocity control? OR simple set power
    public void updateIntakePickupSpeed() {}


    // TODO: Decision -> closed loop kPosition control?
    public void updateIntakePosition() {}


    public void stopIntakeMotors() {}




    public void retractIntake() {}


    public void resetIntake() {}



}
