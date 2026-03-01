package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motors.MotorConfigs;
import org.dyn4j.geometry.hull.GiftWrap;
import com.revrobotics.RelativeEncoder;
import frc.robot.UnitsUtility;

public class IntakeSubsystem extends SubsystemBase {
    // pickup intake motor
    // deploy intake motor(s)
    // Deployed limit switch
    // Retracted limit switch

    private final DigitalInput extendedSwitch1;
    private final DigitalInput extendedSwitch2;
    private final DigitalInput retractedSwitch1;
    private final DigitalInput retractedSwitch2;
    private final RelativeEncoder intakeRelativeEncoder;
    private final SparkMax intakeMotor;
    private final SparkMax deployMotor;

    public IntakeSubsystem(SparkMax intakeMotor,
                           SparkMax deployMotor,
                           DigitalInput extendedSwitch1,
                           DigitalInput extendedSwitch2,
                           DigitalInput retractedSwitch1,
                           DigitalInput retractedSwitch2
                           ){

        this.intakeMotor = intakeMotor;
        this.deployMotor = deployMotor;
        this.extendedSwitch1 = extendedSwitch1;
        this.extendedSwitch2 = extendedSwitch2;
        this.retractedSwitch1 = retractedSwitch1;
        this.retractedSwitch2 = retractedSwitch2;

        this.intakeRelativeEncoder = intakeMotor.getEncoder();



    }

//    public extendIntake() {
//        if (!isIntakeExtended()){
//
//
//        }
//    }
    private boolean intakeOneSwitch(){
        return UnitsUtility.isBeamBroken(extendedSwitch1,false,"Intake Extension Switch 1");
    }


    private boolean intaketTwoSwitch(){
        return UnitsUtility.isBeamBroken(extendedSwitch2,false,"Intake Extension Switch 2");
    }


    private boolean isIntakeExtended(){
        return intakeOneSwitch() && intaketTwoSwitch();
    }



    @Override
    public void periodic() {}


    public void updateIntakeSpeed(){}


    public void stopIntake(){}


    public void resetIntake(){}


    public void shuffleIntake(){}
}
