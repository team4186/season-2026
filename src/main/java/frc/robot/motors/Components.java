package frc.robot.motors;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants.IntakeConstants;


// Flexible motor creation for fast testing between systems
public class Components {
    private static final Components instance = new Components();
    private final MotorConfigs customConfigs = MotorConfigs.getInstance();

    private SparkMax turretMotor;
    private SparkFlex shooterMotor;
    private SparkMax intakePickupMotor;
    private SparkMax spindexerRotateMotor;
    private SparkMax spindexerFeedMotor;

    // private constructor to prevent public class creation
    private Components() { }


    public static Components getInstance() { return instance; }


    public SparkMax getTurretMotor(){
        if ( turretMotor == null ) {
            turretMotor = customConfigs.applyTurretSparkConfig(
                new SparkMax(41, SparkLowLevel.MotorType.kBrushless),
                false
            );
        }

        return turretMotor;
    }


    public SparkFlex getShooterMotor(){
        if (shooterMotor == null) {
            shooterMotor = customConfigs.applyShooterSparkConfig(
                new SparkFlex(21, SparkLowLevel.MotorType.kBrushless),
                new SparkFlex(22, SparkLowLevel.MotorType.kBrushless),
                true,
                false
            );
        }

        return shooterMotor;
    }

    // TODO: Create Motor instance and apply config
    public SparkMax getLoaderMotor(){ return null; }


    // TODO: Check Inverse Condition
    public SparkMax getIntakePickupMotor(){
        if(intakePickupMotor == null) {
            intakePickupMotor = customConfigs.applyIntakePickupSparkConfig(
                    new SparkMax(IntakeConstants.INTAKE_CAN_ID, SparkLowLevel.MotorType.kBrushless),
                    true
            );
        }
        return intakePickupMotor; //67 -S and R
    }


    // TODO: Create Motor instance and apply config
    public SparkMax getIntakeDeployMotor(){ return null; }


    // TODO: Create Motor instance and apply config
    public SparkMax getClimbMotor(){ return null; }


    // TODO: put in correct CAN ID's
    public SparkMax getSpindexerRotateMotor(){
        if (spindexerRotateMotor == null) {
            spindexerRotateMotor = customConfigs.applySpindexerSparkConfig(
                    new SparkMax(0, SparkLowLevel.MotorType.kBrushless),
                    false);
            );
        }
        return spindexerRotateMotor;
    }

    public SparkMax getSpindexerFeedMotor(){
        if (spindexerFeedMotor == null) {
            spindexerFeedMotor = customConfigs.applySpindexerSparkConfig(
                    new SparkMax(0, SparkLowLevel.MotorType.kBrushless),
                    false);
            );
        }
        return spindexerFeedMotor;
}
