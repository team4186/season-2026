package frc.robot.motors;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;


// Flexible motor creation for fast testing between systems
public class Components {
    private static final Components instance = new Components();
    private final MotorConfigs customConfigs = MotorConfigs.getInstance();

    private SparkMax turretMotor;
    private SparkFlex shooterMotor;


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


    // TODO: Create Motor instance and apply config
    public SparkMax getIntakePickupMotor(){ return null; }


    // TODO: Create Motor instance and apply config
    public SparkMax getIntakeDeployMotor(){ return null; }


    // TODO: Create Motor instance and apply config
    public SparkMax getClimbMotor(){ return null; }


    // TODO: Create Motor instance and apply config
    public SparkMax getSpindexerMotor(){ return null; }
}
