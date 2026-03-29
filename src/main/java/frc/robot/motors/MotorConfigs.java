package frc.robot.motors;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.NeoMotorConstants;

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
    private MotorConfigs() {}


    public static MotorConfigs getInstance() { return instance; }


    public SparkBaseConfig applyDefaultSparkMaxConfig() { return DefaultSparkMaxConfig; }


    public SparkBaseConfig applyDefaultSparkFlexConfig() { return DefaultSparkFlexConfig; }


    /**
     * Use this apply config to motor in {@link Components} class.
     *
     * @param motor SparkMax motor object needing configuration
     * @param inverse the motor direction
     *
     * @return SparkMax motor with applied config
     */
    public SparkMax applyTurretRotateSparkConfig(
        SparkMax motor,
        boolean inverse
    ) {
        SparkBaseConfig config = DefaultSparkMaxConfig;

        config
            .inverted(inverse)
            .smartCurrentLimit(Constants.NeoMotorConstants.SMART_CURRENT_LIMIT_550)
            .idleMode(TurretConstants.ROTATE_IDLE_MODE);

        config.encoder
            .positionConversionFactor(TurretConstants.ROTATE_POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(TurretConstants.ROTATE_VELOCITY_CONVERSION_FACTOR);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control. We don't need to pass a closed loop
            // slot, as it will default to slot 0.
            .pid(
                TurretConstants.ROTATE_P,
                TurretConstants.ROTATE_I,
                TurretConstants.ROTATE_D,
                ClosedLoopSlot.kSlot0)
            .outputRange(
                TurretConstants.ROTATE_MIN_OUTPUT,
                TurretConstants.ROTATE_MAX_OUTPUT,
                ClosedLoopSlot.kSlot0)
            .feedForward
            .kS(
                TurretConstants.ROTATE_KS,
                ClosedLoopSlot.kSlot0)
            .kV(
                TurretConstants.ROTATE_KV,
                ClosedLoopSlot.kSlot0);

        motor.configure(
            config,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        return motor;
    }


    // TODO: Implement for individual motor
    public SparkMax applyTurretHoodSparkConfig(
            SparkMax motor,
            boolean inverse
    ) {
        SparkBaseConfig config = DefaultSparkMaxConfig;

        config
                .inverted(inverse)
                .smartCurrentLimit(Constants.NeoMotorConstants.SMART_CURRENT_LIMIT_550)
                .idleMode(TurretConstants.HOOD_IDLE_MODE);

        config.encoder
                .positionConversionFactor(TurretConstants.HOOD_POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(TurretConstants.HOOD_VELOCITY_CONVERSION_FACTOR);

        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control. We don't need to pass a closed loop
                // slot, as it will default to slot 0.
                .pid(
                        TurretConstants.HOOD_P,
                        TurretConstants.HOOD_I,
                        TurretConstants.HOOD_D,
                        ClosedLoopSlot.kSlot0)
                .outputRange(
                        TurretConstants.HOOD_MIN_OUTPUT,
                        TurretConstants.HOOD_MAX_OUTPUT,
                        ClosedLoopSlot.kSlot0)
                .allowedClosedLoopError(
                        TurretConstants.HOOD_ERROR_THRESHOLD,
                        ClosedLoopSlot.kSlot0)
                .feedForward
                .kS(
                        TurretConstants.HOOD_KS,
                        ClosedLoopSlot.kSlot0)
                .kV(
                        TurretConstants.HOOD_KV,
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
        boolean inverse
    ){
        SparkBaseConfig baseConfig = DefaultSparkFlexConfig;

        baseConfig.inverted(inverse)
            .smartCurrentLimit(Constants.NeoMotorConstants.SMART_CURRENT_LIMIT_VORTEX)
            .idleMode(TurretConstants.SHOOTER_IDLE_MODE);

        // Using Velocity
        baseConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(
                    TurretConstants.SHOOTER_P,
                    TurretConstants.SHOOTER_I,
                    TurretConstants.SHOOTER_D,
                    ClosedLoopSlot.kSlot1)
                .outputRange(
                    TurretConstants.SHOOTER_MIN_OUTPUT,
                    TurretConstants.SHOOTER_MAX_OUTPUT,
                    ClosedLoopSlot.kSlot1) // Range of total voltage
                .allowedClosedLoopError(
                        TurretConstants.SHOOTER_ERROR_THRESHOLD,
                        ClosedLoopSlot.kSlot0)
                .feedForward
                .kS(TurretConstants.SHOOTER_KS,
                ClosedLoopSlot.kSlot1)
                .kV(
                TurretConstants.SHOOTER_KV,
                ClosedLoopSlot.kSlot1);

        baseConfig.encoder
            .positionConversionFactor(TurretConstants.SHOOTER_POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(TurretConstants.SHOOTER_VELOCITY_CONVERSION_FACTOR)
            .quadratureAverageDepth(Constants.VELOCITY_AVERAGE_DEPTH)
            .quadratureMeasurementPeriod(Constants.VELOCITY_MEASUREMENT_PERIOD);

        motorLeader.configure(
            baseConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters);


        SparkBaseConfig followerConfig = new SparkFlexConfig();

        followerConfig
            .apply(baseConfig)
            .follow(motorLeader, true);

        motorFollower.configure(
            followerConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters);

        return motorLeader;
    }


    public SparkMax applyIntakeExtensionSparkConfig(
        SparkMax motor,
        boolean inverse
    ) {
            SparkBaseConfig config = DefaultSparkMaxConfig;

            config
                    .inverted(inverse)
                    .smartCurrentLimit(Constants.NeoMotorConstants.SMART_CURRENT_LIMIT_550)
                    .idleMode(IntakeConstants.EXTENSION_IDLE_MODE);

            config.encoder
                    .positionConversionFactor(IntakeConstants.EXTENSION_POSITION_CONVERSION_FACTOR)
                    .velocityConversionFactor(IntakeConstants.EXTENSION_VELOCITY_CONVERSION_FACTOR);

            motor.configure(
                    config,
                    ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters
            );

            return motor;
    }


    public SparkMax applyIntakePickupSparkConfig(
        SparkMax motor,
        boolean inverse
    ) {
            SparkBaseConfig config = DefaultSparkMaxConfig;

            config
                    .inverted(inverse)
                    .smartCurrentLimit(Constants.NeoMotorConstants.SMART_CURRENT_LIMIT_REGULAR)
                    .idleMode(IntakeConstants.PICKUP_IDLE_MODE);

            config.encoder
                    .positionConversionFactor(IntakeConstants.PICKUP_POSITION_CONVERSION_FACTOR)
                    .velocityConversionFactor(IntakeConstants.PICKUP_VELOCITY_CONVERSION_FACTOR);

            motor.configure(
                    config,
                    ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters
            );
            return motor;
    }


    public SparkMax applyClimbSparkConfig(
        SparkMax motor,
        boolean inverse
    ) {
            SparkBaseConfig config = DefaultSparkMaxConfig;

            config
                    .inverted(inverse)
                    .smartCurrentLimit(Constants.NeoMotorConstants.SMART_CURRENT_LIMIT_REGULAR)
                    .idleMode(ClimbConstants.IDLE_MODE);

            config.encoder
                    .positionConversionFactor(ClimbConstants.CLIMB_POSITION_CONVERSION_FACTOR)
                    .velocityConversionFactor(ClimbConstants.CLIMB_VELOCITY_CONVERSION_FACTOR);

            motor.configure(
                    config,
                    ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters
            );

            return motor;
    }


    // TODO: Implement for individual motor
    public SparkMax applySpindexerRotateSparkConfig(
        SparkMax motor,
        boolean inverse
    ) {
            SparkBaseConfig config = DefaultSparkMaxConfig;

            config
                    .inverted(inverse)
                    .smartCurrentLimit(Constants.NeoMotorConstants.SMART_CURRENT_LIMIT_REGULAR)
                    .idleMode(SpindexerConstants.ROTATE_IDLE_MODE);

            config.encoder
                    .positionConversionFactor(SpindexerConstants.ROTATE_POSITION_CONVERSION_FACTOR)
                    .velocityConversionFactor(SpindexerConstants.ROTATE_VELOCITY_CONVERSION_FACTOR);

            motor.configure(
                    config,
                    ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters
            );

            return motor;
    }


    // TODO: Implement for individual motor
    public SparkMax applySpindexerFeedSparkConfig(
        SparkMax motor,
        boolean inverse
    ) {
            SparkBaseConfig config = DefaultSparkMaxConfig;

            config
                    .inverted(inverse)
                    .smartCurrentLimit(SpindexerConstants.FEED_CURRENT_LIMIT)
                    .idleMode(SpindexerConstants.FEED_IDLE_MODE);

            config.encoder
                    .positionConversionFactor(SpindexerConstants.FEED_POSITION_CONVERSION_FACTOR)
                    .velocityConversionFactor(SpindexerConstants.FEED_VELOCITY_CONVERSION_FACTOR);

            motor.configure(
                    config,
                    ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters
            );

            return motor;
    }
}
