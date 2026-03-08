package frc.robot.motors;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TurretConstants;


// Flexible motor creation for fast testing between systems
public class Components {
    private static final Components instance = new Components();
    private final MotorConfigs customConfigs = MotorConfigs.getInstance();

    private SparkMax turretRotateMotor;
    private SparkMax turretHoodMotor;
    private SparkFlex turretShooterMotor;

    private SparkMax intakeExtensionStarboardMotor;
    private SparkMax intakeExtensionPortMotor;

    private SparkMax spindexerRotateMotor;
    private SparkMax spindexerFeedMotor;

    private SparkMax climbMotor;


    // private constructor to prevent public class creation
    private Components() { }


    public static Components getInstance() { return instance; }


    public SparkMax getTurretRotateMotor(){
        if ( turretRotateMotor == null ) {
            turretRotateMotor = customConfigs.applyTurretSparkConfig(
                new SparkMax(TurretConstants.TURRET_ROTATE_MOTOR_ID, SparkLowLevel.MotorType.kBrushless), // TODO: Move id to Constants
                false
            );
        }

        return turretRotateMotor;
    }


    public SparkFlex getTurretShooterMotor(){
        if (turretShooterMotor == null) {
            turretShooterMotor = customConfigs.applyShooterSparkConfig(
                new SparkFlex(TurretConstants.TURRET_SHOOTER_LEAD_MOTOR_ID, SparkLowLevel.MotorType.kBrushless), // TODO: Move id to Constants
                new SparkFlex(TurretConstants.TURRET_SHOOTER_FOLLOWER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless), // TODO: Move id to Constants
                true,
                false
            );
        }

        return turretShooterMotor;
    }


    public SparkMax getTurretHoodMotor(){
        if (turretHoodMotor == null) {
            turretHoodMotor = customConfigs.applyTurretHoodSparkConfig(
                new SparkMax(TurretConstants.TURRET_HOOD_MOTOR_ID, SparkLowLevel.MotorType.kBrushless),
                false
            );
        }

        return turretHoodMotor;
    }


    // TODO: Check Inverse Condition
    public SparkMax intakeExtensionStarboardMotor(){
        if(intakeExtensionStarboardMotor == null) {
            intakeExtensionStarboardMotor = customConfigs.applyIntakePickupSparkConfig(
                    new SparkMax(IntakeConstants.INTAKE_STARBOARD_CAN_ID, SparkLowLevel.MotorType.kBrushless),// TODO: Move id to Constants
                    //new SparkMax(IntakeConstants.INTAKE_CAN_ID, SparkLowLevel.MotorType.kBrushless),
                    true
            );
        }

        return intakeExtensionStarboardMotor; //67 -S and R
    }

    public SparkMax getIntakePickupPortMotor(){
        if(intakeExtensionPortMotor == null) {
            intakeExtensionPortMotor = customConfigs.applyIntakePickupSparkConfig(
                    new SparkMax(IntakeConstants.INTAKE_PORT_CAN_ID, SparkLowLevel.MotorType.kBrushless),// TODO: Move id to Constants
                    //new SparkMax(IntakeConstants.INTAKE_CAN_ID, SparkLowLevel.MotorType.kBrushless),
                    true
            );
        }
        return intakeExtensionPortMotor; //67 -S and R
    }





    // TODO: Create Motor instance and apply config
    public SparkMax getClimbMotor(){
        if (climbMotor == null) {
            climbMotor = customConfigs.applyClimbSparkConfig(
                    new SparkMax(0, SparkLowLevel.MotorType.kBrushless), // TODO: Move id to Constants
                    false);
        }
        return climbMotor;
    }


    // TODO: put in correct CAN ID's
    public SparkMax getSpindexerRotateMotor(){
        if (spindexerRotateMotor == null) {
            spindexerRotateMotor = customConfigs.applySpindexerRotateSparkConfig(
                    new SparkMax(0, SparkLowLevel.MotorType.kBrushless), // TODO: Move id to Constants
                    false);
        }
        return spindexerRotateMotor;
    }

    public SparkMax getSpindexerFeedMotor(){
        if (spindexerFeedMotor == null) {
            spindexerFeedMotor = customConfigs.applySpindexerFeedSparkConfig(
                    new SparkMax(0, SparkLowLevel.MotorType.kBrushless), // TODO: Move id to Constants
                    false);
        }
        return spindexerFeedMotor;
    }
}