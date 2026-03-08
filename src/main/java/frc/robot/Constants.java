// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
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
        public static final String LIMELIGHT_TURRET = "limelight-turret";
        public static final String LIMELIGHT_ROBOT = "limelight-robot";

        // StdDevs for pose estimation trust levels
        public static final double LIMELIGHT_X_STD_DEVS = 0.5;
        public static final double LIMELIGHT_Y_STD_DEVS = 0.5;
        public static final double LIMELIGHT_HEADING_STD_DEVS = 9999999; // StdDevs (x, y, heading)
    }


    public static final class TurretConstants {

        // NEO 550
        public static final int ROTATE_CURRENT_LIMIT = 50;
        public static final double ROTATE_FREE_SPEED = 11000;
        public static final SparkBaseConfig.IdleMode ROTATE_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
        public static final double ROTATE_GEAR_RATIO = 1.0 / 20.0;


        // CAN IDs
        public static final int ROTATE_MOTOR_ID = 41; // TODO: Add equivalent const values and replace in for Components
        public static final int SHOOTER_LEAD_MOTOR_ID = 31;
        public static final int SHOOTER_FOLLOWER_MOTOR_ID = 32;
        public static final int HOOD_MOTOR_ID = 22;

        // Max rotation
        public static final double MAX_ROTATION = 170.0; // Degrees

        // PID // TODO: Update with full weighted system
        public static final double ROTATE_P = 0.0075;
        public static final double ROTATE_I = 0.0;
        public static final double ROTATE_D = 0.002;

        // FeedForward
        public static final double ROTATE_KS = 0.185;
        public static final double ROTATE_KV = NOMINAL_VOLTAGE / TurretConstants.ROTATE_FREE_SPEED;

        public static final double ROTATE_POSITION_CONVERSION_FACTOR = (1 / ROTATE_GEAR_RATIO) * 360; // Convert to degrees
        public static final double ROTATE_VELOCITY_CONVERSION_FACTOR = 1.0;
        public static final double ROTATE_MIN_OUTPUT = -0.75;
        public static final double ROTATE_MAX_OUTPUT = 0.75;



        // NEO Vortex
        public static final int SHOOTER_CURRENT_LIMIT = 50;
        public static final double SHOOTER_FREE_SPEED = 11000;
        public static final SparkBaseConfig.IdleMode SHOOTER_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
        public static final double SHOOTER_GEAR_RATIO = 1.0 / 20.0;


        // PID // TODO: Update with full weighted system
        public static final double SHOOTER_P = 0.0075;
        public static final double SHOOTER_I = 0.0;
        public static final double SHOOTER_D = 0.002;

        // FeedForward
        public static final double SHOOTER_KS = 0.185;
        public static final double SHOOTER_KV = NOMINAL_VOLTAGE / TurretConstants.SHOOTER_FREE_SPEED;

        public static final double SHOOTER_POSITION_CONVERSION_FACTOR = (1 / SHOOTER_GEAR_RATIO) * 360; // Convert to degrees
        public static final double SHOOTER_VELOCITY_CONVERSION_FACTOR = 1.0;
        public static final double SHOOTER_MIN_OUTPUT = -0.75;
        public static final double SHOOTER_MAX_OUTPUT = 0.75;



        // Improving Velocity Based Control
        public static final int SHOOTER_AVERAGE_DEPTH = 5; // 5 Sample Count
        public static final int SHOOTER_MEASUREMENT_PERIOD = 1; // 1ms Moving Avg Window
    }

    public static final class IntakeConstants {

        // NEO 550
        public static final int STARBOARD_EXTENSION_CAN_ID = 0;
        public static final int PORT_EXTENSION_CAN_ID = 0;//TODO: set values here
        public static final int PICKUP_CAN_ID = 0; //TODO: set values here

        //Limit Switches, Extended is when extended, retracted is when retracted, two pairs of switches each, you get it
        public static final int EXTENDED_LSChannel1 = 0; //TODO: set LSChannels
        public static final int EXTENDED_LSChannel2 = 0;
        public static final int RETRACTED_LSChannel1 = 0;
        public static final int RETRACTED_LSChannel2 = 0;

        // Extension current limits & Pickup current limits
        public static final int EXTENSION_CURRENT_LIMIT = 50;
        public static final int PICKUP_CURRENT_LIMIT = 50;

        public static final double EXTENSION_RATIO = 1.0; // TODO: need ratios
        public static final double PICKUP_RATIO = 1.0;

        // Extension Speeds & Pickup Speeds
        public static final double EXTENSION_FREE_SPEED = 11000;
        public static final double PICKUP_FAST_SPEED = 2500; //in rpm, about half of max rpm for a neo brushless
        public static final double PICKUP_SLOW_SPEED = 500; //about 10%

        // Extension idle modes & Pickup idle modes
        public static final SparkBaseConfig.IdleMode EXTENSION_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
        public static final SparkBaseConfig.IdleMode PICKUP_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;

        // Extension PID
        public static final double EXTENSION_P = 0.0; //TODO: set PID values
        public static final double EXTENSION_I = 0.0;
        public static final double EXTENSION_D = 0.0;

        // Extension FeedForward
        public static final double EXTENSION_KS = 0.0; //TODO: set feedforward values
        public static final double EXTENSION_KV = NOMINAL_VOLTAGE / EXTENSION_FREE_SPEED;

        public static final double EXTENSION_POSITION_CONVERSION_FACTOR = 2 * Math.PI * 0.762; // Convert to rev to cm. 0.762 in radius of gear in cm
        public static final double EXTENSION_VELOCITY_CONVERSION_FACTOR = 1.0;
        public static final double EXTENSION_MIN_OUTPUT = -0.75;
        public static final double EXTENSION_MAX_OUTPUT = 0.75;
        public static final double EXTENSION_MAX_ANGLE = 0.00; //TODO: find angles
        public static final double MIN_ANGLE = 0.00;

        // Pickup values
        public static final double PICKUP_POSITION_CONVERSION_FACTOR = 2 * Math.PI * 0.762; // Convert to rev to cm. 0.762 in radius of gear in cm
        public static final double PICKUP_VELOCITY_CONVERSION_FACTOR = 1.0;
        public static final double PICKUP_MIN_OUTPUT = -0.75;
        public static final double PICKUP_MAX_OUTPUT = 0.75;
    }


    public static final class SpindexerConstants {
        //Limit Switches, Extended is when extended, retracted is when retracted, two pairs of switches each, you get it
        public static final int EXTENDED_LSChannel1 = 0;
        public static final int EXTENDED_LSChannel2 = 0;
        public static final int RETRACTED_LSChannel1 = 0;
        public static final int RETRACTED_LSChannel2 = 0;

        // NEO, Feed current limits & rotate current limits
        public static final int FEED_CURRENT_LIMIT = 0;
        public static final int ROTATE_CURRENT_LIMIT = 0;

        // Feed free speeds & Rotate free speeds
        public static final double FEED_FREE_SPEED = 0;
        public static final double ROTATE_FREE_SPEED = 0;

        public static final SparkBaseConfig.IdleMode FEED_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
        public static final SparkBaseConfig.IdleMode ROTATE_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;

        // Feed gear ratios & Rotate gear ratios
        public static final double FEED_GEAR_RATIO = 0;
        public static final double ROTATE_GEAR_RATIO = 0;

        // CAN ID'S
        public static final int ROTATE_CAN_ID = 0;
        public static final int FEED_CAN_ID = 0;


        // Feed PID
        public static final double FEED_P = 0.0075;
        public static final double FEED_I = 0.0;
        public static final double FEED_D = 0.002;

        // Feed FeedForward
        public static final double FEED_KS = 0.185;
        public static final double FEED_KV = NOMINAL_VOLTAGE / FEED_FREE_SPEED;

        public static final double FEED_POSITION_CONVERSION_FACTOR = (1 / FEED_GEAR_RATIO) * 360; // Convert to degrees
        public static final double FEED_VELOCITY_CONVERSION_FACTOR = 1.0;
        public static final double FEED_MIN_OUTPUT = -0.75;
        public static final double FEED_MAX_OUTPUT = 0.75;
        public static final double FEED_MAX_SPEED = 0.75; // set between -1 to 1



        // Rotate PID
        public static final double ROTATE_P = 0.0075;
        public static final double ROTATE_I = 0.0;
        public static final double ROTATE_D = 0.002;

        // Rotate FeedForward
        public static final double ROTATE_KS = 0.185;
        public static final double ROTATE_KV = NOMINAL_VOLTAGE / FEED_FREE_SPEED;

        public static final double ROTATE_POSITION_CONVERSION_FACTOR = (1 / ROTATE_GEAR_RATIO) * 360; // Convert to degrees
        public static final double ROTATE_VELOCITY_CONVERSION_FACTOR = 1.0;
        public static final double ROTATE_MIN_OUTPUT = -0.75;
        public static final double ROTATE_MAX_OUTPUT = 0.75;
        public static final double ROTATE_MAX_SPEED = 0.75; // set between -1 to 1
     }


    // TODO: Update with Constants
    public static final class ClimbConstants {

        public static final int CLIMB_CAN_ID = 0;

        public static final double CLIMB_LSChannel = 0;

        public static final int CLIMB_CURRENT_LIMIT = 0;

        public static final SparkBaseConfig.IdleMode IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;

        public static final double CLIMB_P = 0.0; //TODO: set PID values
        public static final double CLIMB_I = 0.0;
        public static final double CLIMB_D = 0.0;

        public static final double CLIMB_KS = 0.0;
        public static final double CLIMB_KV = 0.0;

        public static final double CLIMB_MIN_OUTPUT = -0.75;
        public static final double CLIMB_MAX_OUTPUT = 0.75;

        public static final double CLIMB_MAX_SPEED = 0.0;

    }

    public static final class NeoMotorConstants {
        // NEO (REGULAR)
        // SMART CURRENT LIMIT (50A - 60A)
        public static final int SMART_CURRENT_LIMIT_REGULAR = 50;
        // NEO 550
        // SMART CURRENT LIMIT (20A - 40A)
        public static final int SMART_CURRENT_LIMIT_550 = 30;
        // NEO VORTEX
        // SMART CURRENT LIMIT 80A
        public static final int SMART_CURRENT_LIMIT_VORTEX = 80;
    }
}
