package frc.robot.motors;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SpindexerConstants;


// MotorConfigs Singleton for Subsystem Motors (Swerve Subsystem not included)
public final class MotorConfigs {
    private static final MotorConfigs instance = new MotorConfigs();

    // Default Configs
    private final SparkBaseConfig DefaultSparkMaxConfig = new SparkMaxConfig()
        .smartCurrentLimit(50)
        .idleMode(SparkBaseConfig.IdleMode.kBrake);

    private final SparkBaseConfig DefaultSparkFlexConfig = new SparkFlexConfig()
        .smartCurrentLimit(50)
        .idleMode(SparkBaseConfig.IdleMode.kBrake);


    // private constructor to prevent public class creation
    private MotorConfigs(){ }


    public static MotorConfigs getInstance() { return instance; }


    public SparkBaseConfig applyDefaultSparkMaxConfig() { return DefaultSparkMaxConfig; }


    public SparkBaseConfig applyDefaultSparkFlexConfig() { return DefaultSparkFlexConfig; }


    // build and return custom config
    public SparkMax applyTurretSparkConfig(
        SparkMax motor,
        boolean inverse
    ) {
        SparkBaseConfig config = DefaultSparkMaxConfig;

        config
            .inverted(inverse)
            .smartCurrentLimit(TurretConstants.MOTOR_CURRENT_LIMIT)
            .idleMode(TurretConstants.IDLE_MODE);

        config.encoder
            .positionConversionFactor(TurretConstants.POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(TurretConstants.VELOCITY_CONVERSION_FACTOR);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control. We don't need to pass a closed loop
            // slot, as it will default to slot 0.
            .pid(
                TurretConstants.TURRET_P,
                TurretConstants.TURRET_I,
                TurretConstants.TURRET_D,
                ClosedLoopSlot.kSlot0)
            .outputRange(
                TurretConstants.MIN_OUTPUT,
                TurretConstants.MAX_OUTPUT,
                ClosedLoopSlot.kSlot0)
            .feedForward
            .kS(
                TurretConstants.TURRET_KS,
                ClosedLoopSlot.kSlot0)
            .kV(
                TurretConstants.TURRET_KV,
                ClosedLoopSlot.kSlot0);

        motor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        return motor;
    }


    public SparkFlex applyShooterSparkConfig(
        SparkFlex motorLeader,
        SparkFlex motorFollower,
        boolean invertSecondMotor,
        boolean inverse
    ){
        SparkBaseConfig baseConfig = DefaultSparkFlexConfig;

        baseConfig.inverted(inverse)
            .smartCurrentLimit(ShooterConstants.MOTOR_CURRENT_LIMIT)
            .idleMode(ShooterConstants.IDLE_MODE);

        // Using Velocity
        baseConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(
                ShooterConstants.SHOOTER_P,
                ShooterConstants.SHOOTER_I,
                ShooterConstants.SHOOTER_D,
                ClosedLoopSlot.kSlot1)
            .outputRange(
                ShooterConstants.MIN_OUTPUT,
                ShooterConstants.MAX_OUTPUT,
                ClosedLoopSlot.kSlot1) // Range of total voltage
            .feedForward
            // kV is now in Volts, so we multiply by the nominal voltage (12V)
            .kS(ShooterConstants.SHOOTER_KS,
                ClosedLoopSlot.kSlot1)
            .kV(
                ShooterConstants.SHOOTER_KV,
                ClosedLoopSlot.kSlot1);

        baseConfig.encoder
            .positionConversionFactor(ShooterConstants.POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(ShooterConstants.VELOCITY_CONVERSION_FACTOR)
            .quadratureAverageDepth(ShooterConstants.AVERAGE_DEPTH)
            .quadratureMeasurementPeriod(ShooterConstants.MEASUREMENT_PERIOD); // NOTE TO SELF: DOUBLE CHECK

        motorLeader.configure(
            baseConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters);


        SparkBaseConfig followerConfig = new SparkFlexConfig();

        // boolean invertFollower = invertSecondMotor ^ inverse; // using XOR boolean logic
        boolean invertFollower = inverse;
        if (invertSecondMotor) {
            invertFollower = !invertFollower;
        }

        followerConfig
            .apply(baseConfig)
            .follow(motorLeader, invertFollower);

        motorFollower.configure(
            followerConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters);

        return motorLeader;
    }


    // TODO: Setup SparkMax config
    public SparkMax applyIntakePickupSparkConfig(
        SparkMax motor,
        boolean inverse
    ) {
            SparkBaseConfig config = DefaultSparkMaxConfig;

            config
                    .inverted(inverse)
                    .smartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT)
                    .idleMode(IntakeConstants.IDLE_MODE);

            config.encoder
                    .positionConversionFactor(IntakeConstants.POSITION_CONVERSION_FACTOR)
                    .velocityConversionFactor(IntakeConstants.VELOCITY_CONVERSION_FACTOR);

            config.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // Set PID values for position control. We don't need to pass a closed loop
                    // slot, as it will default to slot 0.
                    .pid(
                            IntakeConstants.INTAKE_PICKUP_P,
                            IntakeConstants.INTAKE_PICKUP_I,
                            IntakeConstants.INTAKE_PICKUP_D,
                            ClosedLoopSlot.kSlot0)
                    .outputRange(
                            IntakeConstants.MIN_OUTPUT,
                            IntakeConstants.MAX_OUTPUT,
                            ClosedLoopSlot.kSlot0)
                    .feedForward
                    .kS(
                            IntakeConstants.INTAKE_PICKUP_KS,
                            ClosedLoopSlot.kSlot0)
                    .kV(
                            IntakeConstants.INTAKE_PICKUP_KV,
                            ClosedLoopSlot.kSlot0);

            motor.configure(
                    config,
                    ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters
            );

            return motor;
    }

    // TODO: Setup SparkMax config
    public SparkMax applyIntakeDeployerSparkConfig(){
        return null;
    }


    // TODO: Setup SparkMax config
    public SparkMax applyClimbSparkConfig(){
        return null;
    }


    // TODO: Setup SparkMax config
    public SparkMax applyTurretLoaderSparkConfig(){
        return null;
    }


    // TODO: Setup SparkMax config
    public SparkMax applySpindexerSparkConfig(
        SparkMax motor,
        boolean inverse
    ) {
            SparkBaseConfig config = DefaultSparkMaxConfig;

            config
                    .inverted(inverse)
                    .smartCurrentLimit(SpindexerConstants.SPINDEXER_CURRENT_LIMIT)
                    .idleMode(SpindexerConstants.IDLE_MODE);

            config.encoder
                    .positionConversionFactor(SpindexerConstants.POSITION_CONVERSION_FACTOR)
                    .velocityConversionFactor(SpindexerConstants.VELOCITY_CONVERSION_FACTOR);

            // Add if we decide to use PIDS for spindexer, instead of feeding a velocity - Shing
//            config.closedLoop
//                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
//                    // Set PID values for position control. We don't need to pass a closed loop
//                    // slot, as it will default to slot 0.
//                    .pid(
//                            IntakeConstants.INTAKE_PICKUP_P,
//                            IntakeConstants.INTAKE_PICKUP_I,
//                            IntakeConstants.INTAKE_PICKUP_D,
//                            ClosedLoopSlot.kSlot0)
//                    .outputRange(
//                            IntakeConstants.MIN_OUTPUT,
//                            IntakeConstants.MAX_OUTPUT,
//                            ClosedLoopSlot.kSlot0)
//                    .feedForward
//                    .kS(
//                            IntakeConstants.INTAKE_PICKUP_KS,
//                            ClosedLoopSlot.kSlot0)
//                    .kV(
//                            IntakeConstants.INTAKE_PICKUP_KV,
//                            ClosedLoopSlot.kSlot0);

            motor.configure(
                    config,
                    ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters
            );

            return motor;
    }
}
