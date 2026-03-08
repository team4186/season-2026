package frc.robot.motors;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.Constants.ClimbConstants;


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


    // build and return custom config
    public SparkMax applyTurretRotateSparkConfig(
        SparkMax motor,
        boolean inverse
    ) {
        SparkBaseConfig config = DefaultSparkMaxConfig;

        config
            .inverted(inverse)
            .smartCurrentLimit(TurretConstants.ROTATE_CURRENT_LIMIT)
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

    public SparkMax applyTurretShooterSparkConfig(
            SparkMax motor,
            boolean inverse
    ) {
        SparkBaseConfig config = DefaultSparkMaxConfig;

        config
                .inverted(inverse)
                .smartCurrentLimit(TurretConstants.SHOOTER_CURRENT_LIMIT)
                .idleMode(TurretConstants.SHOOTER_IDLE_MODE);

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


    public SparkFlex applyShooterSparkConfig(
        SparkFlex motorLeader,
        SparkFlex motorFollower,
        boolean invertSecondMotor,
        boolean inverse
    ){
        SparkBaseConfig baseConfig = DefaultSparkFlexConfig;

        baseConfig.inverted(inverse)
            .smartCurrentLimit(TurretConstants.SHOOTER_CURRENT_LIMIT)
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
            .feedForward
            // kV is now in Volts, so we multiply by the nominal voltage (12V)
            .kS(TurretConstants.SHOOTER_KS,
                ClosedLoopSlot.kSlot1)
            .kV(
                TurretConstants.SHOOTER_KV,
                ClosedLoopSlot.kSlot1);

        baseConfig.encoder
            .positionConversionFactor(TurretConstants.SHOOTER_POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(TurretConstants.SHOOTER_VELOCITY_CONVERSION_FACTOR)
            .quadratureAverageDepth(TurretConstants.SHOOTER_AVERAGE_DEPTH)
            .quadratureMeasurementPeriod(TurretConstants.SHOOTER_MEASUREMENT_PERIOD); // NOTE TO SELF: DOUBLE CHECK

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
    public SparkMax applyIntakeExtensionSparkConfig(
        SparkMax motor,
        boolean inverse
    ) {
            SparkBaseConfig config = DefaultSparkMaxConfig;

            config
                    .inverted(inverse)
                    .smartCurrentLimit(IntakeConstants.EXTENSION_CURRENT_LIMIT)
                    .idleMode(IntakeConstants.EXTENSION_IDLE_MODE);

            config.encoder
                    .positionConversionFactor(IntakeConstants.EXTENSION_POSITION_CONVERSION_FACTOR)
                    .velocityConversionFactor(IntakeConstants.EXTENSION_VELOCITY_CONVERSION_FACTOR);

            config.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // Set PID values for position control. We don't need to pass a closed loop
                    // slot, as it will default to slot 0.
                    .pid(
                            IntakeConstants.EXTENSION_P,
                            IntakeConstants.EXTENSION_I,
                            IntakeConstants.EXTENSION_D,
                            ClosedLoopSlot.kSlot0)
                    .outputRange(
                            IntakeConstants.EXTENSION_MIN_OUTPUT,
                            IntakeConstants.EXTENSION_MAX_OUTPUT,
                            ClosedLoopSlot.kSlot0)
                    .feedForward
                    .kS(
                            IntakeConstants.EXTENSION_KS,
                            ClosedLoopSlot.kSlot0)
                    .kV(
                            IntakeConstants.EXTENSION_KV,
                            ClosedLoopSlot.kSlot0);

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
                    .smartCurrentLimit(IntakeConstants.PICKUP_CURRENT_LIMIT)
                    .idleMode(IntakeConstants.PICKUP_IDLE_MODE);

            config.encoder
                    .positionConversionFactor(IntakeConstants.PICKUP_POSITION_CONVERSION_FACTOR)
                    .velocityConversionFactor(IntakeConstants.PICKUP_VELOCITY_CONVERSION_FACTOR);

        //Add if we decide to use closed loop for the intake pickup - Shing
//            config.closedLoop
//                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
//                    // Set PID values for position control. We don't need to pass a closed loop
//                    // slot, as it will default to slot 0.
//                    .pid(
//                            IntakeConstants.INTAKE_P,
//                            IntakeConstants.INTAKE_I,
//                            IntakeConstants.INTAKE_D,
//                            ClosedLoopSlot.kSlot0)
//                    .outputRange(
//                            IntakeConstants.INTAKE_MIN_OUTPUT,
//                            IntakeConstants.INTAKE_MAX_OUTPUT,
//                            ClosedLoopSlot.kSlot0)
//                    .feedForward
//                    .kS(
//                            IntakeConstants.INTAKE_KS,
//                            ClosedLoopSlot.kSlot0)
//                    .kV(
//                            IntakeConstants.INTAKE_KV,
//                            ClosedLoopSlot.kSlot0);

            motor.configure(
                    config,
                    ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters
            );
            return motor;
    }



    // TODO: Setup SparkMax config
    public SparkMax applyClimbSparkConfig(
        SparkMax motor,
        boolean inverse
    ) {
            SparkBaseConfig config = DefaultSparkMaxConfig;

            config
                    .inverted(inverse)
                    .smartCurrentLimit(ClimbConstants.CLIMB_CURRENT_LIMIT)
                    .idleMode(ClimbConstants.IDLE_MODE);

//            config.encoder  //not sur ehow encoder conversions work yet
//                    .positionConversionFactor(ClimbConstants.INTAKE_POSITION_CONVERSION_FACTOR)
//                    .velocityConversionFactor(ClimbConstants.INTAKE_VELOCITY_CONVERSION_FACTOR);

            config.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // Set PID values for position control. We don't need to pass a closed loop
                    // slot, as it will default to slot 0.
                    .pid(
                            ClimbConstants.CLIMB_P,
                            ClimbConstants.CLIMB_I,
                            ClimbConstants.CLIMB_D,
                            ClosedLoopSlot.kSlot0)
                    .outputRange(
                            ClimbConstants.CLIMB_MIN_OUTPUT,
                            ClimbConstants.CLIMB_MAX_OUTPUT,
                            ClosedLoopSlot.kSlot0)
                    .feedForward
                    .kS(
                            ClimbConstants.CLIMB_KS,
                            ClosedLoopSlot.kSlot0)
                    .kV(
                            ClimbConstants.CLIMB_KV,
                            ClosedLoopSlot.kSlot0);

            motor.configure(
                    config,
                    ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters
            );

            return motor;
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
                    .smartCurrentLimit(SpindexerConstants.CURRENT_LIMIT)
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


    // TODO: Implement for individual motor
    public SparkMax applySpindexerFeedSparkConfig(SparkMax sparkMax, boolean inverse) {
        return null;
    }


    // TODO: Implement for individual motor
    public SparkMax applySpindexerRotateSparkConfig(SparkMax sparkMax, boolean inverse) {
        return null;
    }


    // TODO: Implement for individual motor
    public SparkMax applyTurretHoodSparkConfig(SparkMax sparkMax, boolean inverse) { return null; }
}
