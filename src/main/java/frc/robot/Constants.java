// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED = Units.feetToMeters(18.0); //orig value 14.5
    public static final double NOMINAL_VOLTAGE = 12.0;

    // Maximum speed of the robot in meters per second, used to limit acceleration.

     public static final class AutonConstants {
//         public static final PIDConstants TRANSLATION_PID = new PIDConstants(
//                 0.7,
//                 0,
//                 0);
//         public static final PIDConstants ANGLE_PID = new PIDConstants(
//                 0.4,
//                 0,
//                 0.01);
     }


    public static final class DrivebaseConstants {
        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds
    }


    public static class OperatorConstants {
        // Joystick Deadband
        public static final double DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT = 6;
    }


    public static class LimelightConstants {
        // Camera names
        public static final String LIMELIGHT_ROBOT = "limelight-turret";
        // public static final String LIMELIGHT_ROBOT = "limelight-robot";
        public static final String LIMELIGHT_TURRET = "limelight-turret";

        // StdDevs for pose estimation trust levels
        public static final double LIMELIGHT_X_STD_DEVS = 0.5;
        public static final double LIMELIGHT_Y_STD_DEVS = 0.5;
        public static final double LIMELIGHT_HEADING_STD_DEVS = 9999999; // StdDevs (x, y, heading)
    }


    public static final class TurretConstants {
        // NEO 550
        public static final int MOTOR_CURRENT_LIMIT = 50;
        public static final double MOTOR_FREE_SPEED = 11000;
        public static final SparkBaseConfig.IdleMode IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
        public static final double GEAR_RATIO = 1.0 / 20.0;

        // Max rotation
        public static final double TURRET_MAX_ROTATION = 170;

        // PID
        public static final double TURRET_P = 0.0075;
        public static final double TURRET_I = 0.0;
        public static final double TURRET_D = 0.002;

        // FeedForward
        public static final double TURRET_KS = 0.185;
        public static final double TURRET_KV = NOMINAL_VOLTAGE / TurretConstants.MOTOR_FREE_SPEED;

        public static final double POSITION_CONVERSION_FACTOR = (1 / GEAR_RATIO) * 360; // Convert to degrees
        public static final double VELOCITY_CONVERSION_FACTOR = 1.0;
        public static final double MIN_OUTPUT = -0.75;
        public static final double MAX_OUTPUT = 0.75;
    }


    public static final class ShooterConstants {
        // NEO Vortex
        public static final SparkBaseConfig.IdleMode IDLE_MODE = SparkBaseConfig.IdleMode.kCoast;
        public static final int MOTOR_CURRENT_LIMIT = 80; // NEO VORTEX
        public static final double MOTOR_FREE_SPEED = 6784.0;

        // PID
        public static final double SHOOTER_P = 0.001;
        public static final double SHOOTER_I = 0.0;
        public static final double SHOOTER_D = 0.0;

        // FeedForward
        public static final double SHOOTER_KS = 0.10;
        public static final double SHOOTER_KV = NOMINAL_VOLTAGE / ShooterConstants.MOTOR_FREE_SPEED;

        // CLOSED LOOP CONTROLLER
        public static final ControlType CONTROL_TYPE = ControlType.kVelocity;
        public static final double POSITION_CONVERSION_FACTOR = 1.0;
        public static final double VELOCITY_CONVERSION_FACTOR = 1.0;
        public static final double MIN_OUTPUT = -0.90;
        public static final double MAX_OUTPUT = 0.90;

        // Improving Velocity Based Control
        public static final int AVERAGE_DEPTH = 5; // 5 Sample Count
        public static final int MEASUREMENT_PERIOD = 1; // 1ms Moving Avg Window
    }



    public static final class IntakeConstants {
        //Limit Switches, Extended is when extended, retracted is when retracted, two pairs of switches each, you get it
         public static final int INTAKE_EXTENDED_LSChannel1 = 0; //TODO: set LSChannels
        public static final int INTAKE_EXTENDED_LSChannel2 = 0;
        public static final int INTAKE_RETRACTED_LSChannel1 = 0;
        public static final int INTAKE_RETRACTED_LSChannel2 = 0;

        // NEO 550

        public static final int INTAKE_CAN_ID = 0; //TODO: set values here
        public static final int INTAKE_CURRENT_LIMIT = 50;
        public static final double INTAKE_FREE_SPEED = 11000;
        public static final SparkBaseConfig.IdleMode IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
        public static final double INTAKE_RATIO = 1.0; // slightly different in reality but who cares

        // PID
        public static final double INTAKE_P = 0.0; //TODO: set PID values
        public static final double INTAKE_I = 0.0;
        public static final double INTAKE_D = 0.0;

        // FeedForward
        public static final double INTAKE_KS = 0.0; //TODO: set feedforward values
        public static final double INTAKE_KV = NOMINAL_VOLTAGE / INTAKE_FREE_SPEED;


        public static final double INTAKE_POSITION_CONVERSION_FACTOR = (1 / INTAKE_RATIO) * 360; // Convert to degrees
        public static final double INTAKE_VELOCITY_CONVERSION_FACTOR = 1.0;
        public static final double INTAKE_MIN_OUTPUT = -0.75;
        public static final double INTAKE_MAX_OUTPUT = 0.75;
        public static final double INTAKE_MAX_ANGLE = 0.00; //TODO: find angles
        public static final double MIN_ANGLE = 0.00;



        public static final int DEPLOY_CAN_ID = 0; //TODO: set values here
        public static final int DEPLOY_CURRENT_LIMIT = 50;
        public static final double DEPLOY_SPEED = 0; //still in rpm, might wanna change this
        public static final double DEPLOY_RATIO = 1.0; // need to figure this out

        public static final double DEPLOY_PICKUP_P = 0.0; //TODO: set PID values
        public static final double DEPLOY_PICKUP_I = 0.0;
        public static final double DEPLOY_PICKUP_D = 0.0;

        // FeedForward
        public static final double DEPLOY_PICKUP_KS = 0.0; //TODO: set feedforward values
        public static final double DEPLOY_PICKUP_KV = NOMINAL_VOLTAGE / INTAKE_FREE_SPEED;


        public static final double DEPLOY_MIN_OUTPUT = -0.75;
        public static final double DEPLOY_MAX_OUTPUT = 0.75;
        public static final double DEPLOY_MAX_ANGLE = 0.00; //TODO: find angles
        public static final double DEPLOY_MIN_ANGLE = 0.00;
    }

    public static final class SpindexerConstants {
        //Limit Switches, Extended is when extended, retracted is when retracted, two pairs of switches each, you get it
        public static final int SPINDEXER_EXTENDED_LSChannel1 = 0;
        public static final int SPINDEXER_EXTENDED_LSChannel2 = 0;
        public static final int SPINDEXER_RETRACTED_LSChannel1 = 0;
        public static final int SPINDEXER_RETRACTED_LSChannel2 = 0;

        // NEO 550

        public static final int SPINDEXER_CAN_ID = 50;
        public static final int SPINDEXER_CURRENT_LIMIT = 50;
        public static final double SPINDEXER_FREE_SPEED = 11000;
        public static final SparkBaseConfig.IdleMode IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
        public static final double GEAR_RATIO = 1.0; // probably a little different but who cares

        // PID
        public static final double SPINDEXER_PICKUP_P = 0.0075;
        public static final double SPINDEXER_PICKUP_I = 0.0;
        public static final double SPINDEXER_PICKUP_D = 0.002;

        // FeedForward
        public static final double SPINDEXER_PICKUP_KS = 0.185;
        public static final double SPINDEXER_PICKUP_KV = NOMINAL_VOLTAGE / SPINDEXER_FREE_SPEED;

        public static final double POSITION_CONVERSION_FACTOR = (1 / GEAR_RATIO) * 360; // Convert to degrees
        public static final double VELOCITY_CONVERSION_FACTOR = 1.0;
        public static final double MIN_OUTPUT = -0.75;
        public static final double MAX_OUTPUT = 0.75;
        public static final double MAX_SPEED = 0.75; // set between -1 to 1
    }

    // TODO: Update with Constants
    public static final class ClimbConstants {}
}
