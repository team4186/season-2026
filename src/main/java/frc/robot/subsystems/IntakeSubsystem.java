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
    // pickup intake motor
    // deploy intake motor(s)
    // Deployed limit switch
    // Retracted limit switch

    private final DigitalInput extendedSwitch1;
    private final DigitalInput extendedSwitch2;
    private final DigitalInput retractedSwitch1;
    private final DigitalInput retractedSwitch2;
    private final RelativeEncoder extensionRelativeEncoder;
    private final SparkMax intakeMotor;
    private final SparkMax extendMotor;
    private final PIDController extensionPIDController;

    public IntakeSubsystem(SparkMax intakeMotor,
                           SparkMax extendMotor,
                           DigitalInput extendedSwitch1,
                           DigitalInput extendedSwitch2,
                           DigitalInput retractedSwitch1,
                           DigitalInput retractedSwitch2,
                           PIDController extensionPIDController
    ) {

        this.intakeMotor = intakeMotor;
        this.extendMotor = extendMotor;
        this.extendedSwitch1 = extendedSwitch1;
        this.extendedSwitch2 = extendedSwitch2;
        this.retractedSwitch1 = retractedSwitch1;
        this.retractedSwitch2 = retractedSwitch2;
        this.extensionPIDController = extensionPIDController;
        this.extensionRelativeEncoder = intakeMotor.getEncoder();


    }


    private boolean intakeOneSwitch() {
        return UnitsUtility.isBeamBroken(extendedSwitch1, false, "Intake Extension Switch 1");
    }


    private boolean intakeTwoSwitch() {
        return UnitsUtility.isBeamBroken(extendedSwitch2, false, "Intake Extension Switch 2");
    }


    private boolean isIntakeExtended() {
        return intakeOneSwitch() && intakeTwoSwitch();
    }


    @Override
    public void periodic() {}


    public void updateIntakeSpeed() {}


    public void stopIntake() {}


    public void extendIntake() {}

    public void retractIntake() {}
}
