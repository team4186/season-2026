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
    private final SparkMax extendMotor;

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
        this.extendMotor = extendMotor; // TODO: Decision -> Leader/Follower or 2 separate motors given similar power / goals

        this.extendedSwitchStarboard = extendedSwitch1;
        this.extendedSwitchPort = extendedSwitch2;
        this.retractedSwitchStarboard = retractedSwitch1;
        this.retractedSwitchPort = retractedSwitch2;

        // TODO: Do we want to replace with a closed loop controller for this?


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
        return UnitsUtility.isBeamBroken(extendedSwitchStarboard, false, "Intake Extension Switch 1");
    }


    private boolean isPortExtended() {
        return UnitsUtility.isBeamBroken(extendedSwitchPort, false, "Intake Extension Switch 2");
    }


    // TODO: Question --- Does the motor pair setup change anything here?
    private boolean isIntakeExtended() {
        return isStarboardExtended() && isPortExtended();
    }




    private void extendIntakeStarboard(){
        if(!isStarboardExtended()){
            extensionStarboardController.setSetpoint(31.0, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
        } else {
            extensionStarboardMotor.stopMotor();
        }
    }
    //TODO: MAKE SURE THE MOTORS ARE INVERTED,AND OPPOSITE CORRECTLY!!!!!! VERY IMPORTANT OR STUFF WILL BREAK!!!!! -Shing
    private void extendIntakePort(){
        if(!isPortExtended()){
            extensionPortController.setSetpoint(31.0, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot1);
        } else {
            extensionPortMotor.stopMotor();
        }
    }

    // TODO: Decision -> closed loop kVelocity control? OR simple set power
    public void updateIntakePickupSpeed() {}


    // TODO: Decision -> closed loop kPosition control?
    public void updateIntakePosition() {}


    public void stopIntakeMotors() {}


    public void extendIntake() {}


    public void retractIntake() {}


    public void resetIntake() {}

    public double currentPosition(RelativeEncoder relativeEncoder){
        return relativeEncoder.getPosition()*2*3.141592*0.762;
        //0.3in is the radius of the gears = 0.726cm
    }

}
