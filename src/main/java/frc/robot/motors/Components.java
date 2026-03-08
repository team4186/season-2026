package frc.robot.motors;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.SpindexerConstants;

// Flexible motor creation for fast testing between systems
public class Components {
    private static final Components instance = new Components();
    private final MotorConfigs customConfigs = MotorConfigs.getInstance();

    private SparkMax turretRotateMotor;
    private SparkMax turretHoodMotor;
    private SparkFlex turretShooterMotor;

    private SparkMax intakeExtensionStarboardMotor;
    private SparkMax intakeExtensionPortMotor;
    private SparkMax intakePickupMotor;

    private SparkMax spindexerRotateMotor;
    private SparkMax spindexerFeedMotor;

    private SparkMax climbMotor;


    // private constructor to prevent public class creation
    private Components() { }


    public static Components getInstance() { return instance; }


    public SparkMax getTurretRotateMotor(){
        if ( turretRotateMotor == null ) {
            turretRotateMotor = customConfigs.applyTurretRotateSparkConfig(
                new SparkMax(TurretConstants.ROTATE_MOTOR_ID, SparkLowLevel.MotorType.kBrushless), // TODO: Move id to Constants
                false
            );
        }

        return turretRotateMotor;
    }


    public SparkFlex getTurretShooterMotor(){
        if (turretShooterMotor == null) {
            turretShooterMotor = customConfigs.applyShooterSparkConfig(
                new SparkFlex(TurretConstants.SHOOTER_LEAD_MOTOR_ID, SparkLowLevel.MotorType.kBrushless), // TODO: Move id to Constants
                new SparkFlex(TurretConstants.SHOOTER_FOLLOWER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless), // TODO: Move id to Constants
                true,
                false
            );
        }

        return turretShooterMotor;
    }


    public SparkMax getTurretHoodMotor(){
        if (turretHoodMotor == null) {
            turretHoodMotor = customConfigs.applyTurretHoodSparkConfig(
                new SparkMax(TurretConstants.HOOD_MOTOR_ID, SparkLowLevel.MotorType.kBrushless),
                false
            );
        }

        return turretHoodMotor;
    }


    // TODO: Check Inverse Condition
    public SparkMax intakeExtensionStarboardMotor(){
        if(intakeExtensionStarboardMotor == null) {
            intakeExtensionStarboardMotor = customConfigs.applyIntakeExtensionSparkConfig(
                    new SparkMax(IntakeConstants.STARBOARD_EXTENSION_CAN_ID, SparkLowLevel.MotorType.kBrushless),// TODO: Move id to Constants
                    //new SparkMax(IntakeConstants.INTAKE_CAN_ID, SparkLowLevel.MotorType.kBrushless),
                    true
            );
        }
        return intakeExtensionStarboardMotor; //67 -S and R
    }

    public SparkMax getIntakeExtensionPortMotor(){
        if(intakeExtensionPortMotor == null) {
            intakeExtensionPortMotor = customConfigs.applyIntakeExtensionSparkConfig(
                    new SparkMax(IntakeConstants.PORT_EXTENSION_CAN_ID, SparkLowLevel.MotorType.kBrushless),// TODO: Move id to Constants
                    //new SparkMax(IntakeConstants.INTAKE_CAN_ID, SparkLowLevel.MotorType.kBrushless),
                    true
            );
        }
        return intakeExtensionPortMotor; //67 -S and R
    }

    public SparkMax getIntakePickupMotor(){
        if(intakePickupMotor == null){
            intakePickupMotor = customConfigs.applyIntakePickupSparkConfig(
                    new SparkMax(IntakeConstants.PICKUP_CAN_ID,
                            SparkLowLevel.MotorType.kBrushless),
                    true
            );
        }
        return intakePickupMotor;
    }




    // TODO: Create Motor instance and apply config
    public SparkMax getClimbMotor(){
        if (climbMotor == null) {
            climbMotor = customConfigs.applyClimbSparkConfig(
                    new SparkMax(ClimbConstants.CLIMB_CAN_ID, SparkLowLevel.MotorType.kBrushless), // TODO: Move id to Constants
                    false);
        }
        return climbMotor;
    }


    // TODO: put in correct CAN ID's
    public SparkMax getSpindexerRotateMotor(){
        if (spindexerRotateMotor == null) {
            spindexerRotateMotor = customConfigs.applySpindexerRotateSparkConfig(
                    new SparkMax(SpindexerConstants.ROTATE_CAN_ID, SparkLowLevel.MotorType.kBrushless), // TODO: Move id to Constants
                    false);
        }
        return spindexerRotateMotor;
    }

    public SparkMax getSpindexerFeedMotor(){
        if (spindexerFeedMotor == null) {
            spindexerFeedMotor = customConfigs.applySpindexerFeedSparkConfig(
                    new SparkMax(SpindexerConstants.FEED_CAN_ID, SparkLowLevel.MotorType.kBrushless), // TODO: Move id to Constants
                    false);
        }
        return spindexerFeedMotor;
    }
}