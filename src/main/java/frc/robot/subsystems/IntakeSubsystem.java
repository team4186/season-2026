package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;
import frc.robot.UnitsUtility;
import java.lang.invoke.ConstantCallSite;


public class IntakeSubsystem extends SubsystemBase {

    private final DigitalInput extendedSwitch1;
    private final DigitalInput extendedSwitch2;
    private final DigitalInput retractedSwitch1;
    private final DigitalInput retractedSwitch2;

    private final RelativeEncoder extensionRelativeEncoder;

    private final SparkMax intakeMotor;
    private final SparkMax extendMotor;

    private final PIDController extensionPIDController;


    public IntakeSubsystem(
            SparkMax intakeMotor,
            SparkMax extendMotor,
            DigitalInput extendedSwitch1,
            DigitalInput extendedSwitch2,
            DigitalInput retractedSwitch1,
            DigitalInput retractedSwitch2,
            PIDController extensionPIDController
    ) {
        this.intakeMotor = intakeMotor;
        this.extendMotor = extendMotor; // TODO: Decision -> Leader/Follower or 2 separate motors given similar power / goals

        this.extendedSwitch1 = extendedSwitch1;
        this.extendedSwitch2 = extendedSwitch2;
        this.retractedSwitch1 = retractedSwitch1;
        this.retractedSwitch2 = retractedSwitch2;

        // TODO: Do we want to replace with a closed loop controller for this?
        this.extensionPIDController = extensionPIDController;
        this.extensionRelativeEncoder = intakeMotor.getEncoder();
    }


    // TODO: What can we publish from this subsystem for system checks and tracking the robot state
    @Override
    public void periodic() {}


    // TODO: Lets rename this function to describe its use
    private boolean intakeOneSwitch() {
        return UnitsUtility.isBeamBroken(extendedSwitch1, false, "Intake Extension Switch 1");
    }


    // TODO: Lets rename this function to describe its use
    private boolean intakeTwoSwitch() {
        return UnitsUtility.isBeamBroken(extendedSwitch2, false, "Intake Extension Switch 2");
    }


    // TODO: Question --- Does the motor pair setup change anything here?
    private boolean isIntakeExtended() {
        return intakeOneSwitch() && intakeTwoSwitch();
    }


    // TODO: Decision -> closed loop kVelocity control? OR simple set power
    public void updateIntakePickupSpeed() {}


    // TODO: Decision -> closed loop kPosition control?
    public void updateIntakePosition() {}


    public void stopIntakeMotors() {}


    public void extendIntake() {}


    public void retractIntake() {}


    public void resetIntake() {}
}
