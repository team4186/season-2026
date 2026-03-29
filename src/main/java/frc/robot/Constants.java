// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import swervelib.math.Matter;
import com.revrobotics.spark.config.SparkBaseConfig;

import java.util.HashMap;
import java.util.Map;

import static edu.wpi.first.units.Units.Meter;
import static java.util.Map.entry;


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

    // public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final double ROBOT_MASS = (112.72) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag // TODO: Modify?
    public static final double MAX_SPEED = Units.feetToMeters(13.76); //orig value 14.5
    public static final double NOMINAL_VOLTAGE = 12.0;

    // Improving Velocity Based Control
    public static final int VELOCITY_AVERAGE_DEPTH = 5; // 5 Sample Count
    public static final int VELOCITY_MEASUREMENT_PERIOD = 1; // 1ms Moving Avg Window

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
            //Left and right are relative to robot looking at tags.
    }


    public static final class StructureConstants {
        //TODO Constants: Tag ids for red and blue side, and static locations in meters of center of structure
        public static final int[] BLUE_FIDUCIAL_TURRET_IDS = { 18, 19, 20, 21, 24, 25, 26, 27 };
        public static final int[] RED_FIDUCIAL_TURRET_IDS =  { 2, 3, 4, 5, 8, 9, 10, 11 };

        // OFFSETS Offset to the side, back is the same regardless, height is always the same, +- 14 inches
        // TODO: Setup at start of match for pipeline 1, 2, 3 (center, left, right)
        public static final double TURRET_TARGET_FORWARD_OFFSET = Units.inchesToMeters(-23.51);

        public static final double TURRET_TARGET_RIGHT_SIDE_OFFSET = Units.inchesToMeters(14.0);
        public static final double TURRET_TARGET_LEFT_SIDE_OFFSET = -TURRET_TARGET_RIGHT_SIDE_OFFSET;

        public static final int[] OFFSET_GROUP_CENTER_RED = { 2, 4, 5, 10 };
        public static final int[] OFFSET_GROUP_LEFT_RED = { 3, 9, 11 };
        public static final int[] OFFSET_GROUP_RIGHT_RED = { 8 };

        public static final int[] OFFSET_GROUP_CENTER_BLUE = { 18, 20, 21, 26 };
        public static final int[] OFFSET_GROUP_LEFT_BLUE = { 19, 25, 27 };
        public static final int[] OFFSET_GROUP_RIGHT_BLUE = { 24 };

        // Key: Turret Tag Id, Value: POI RightOffset
        public static final Map<Integer, Integer> TURRET_FIDUCIAL_PIPELINE = new HashMap<>(
            Map.ofEntries(
                entry(2, 1 ), // 0.0
                entry(3, 2 ), // TURRET_TARGET_LEFT_SIDE_OFFSET
                entry(4, 1 ), // 0.0
                entry(5, 1 ), //0.0),
                entry(8, 3 ), //TURRET_TARGET_RIGHT_SIDE_OFFSET),
                entry(9, 2 ), //TURRET_TARGET_LEFT_SIDE_OFFSET),
                entry(10, 1 ), //0.0),
                entry(11, 2 ), //TURRET_TARGET_LEFT_SIDE_OFFSET),
                entry(18, 1 ), //0.0),
                entry(19, 2 ), //TURRET_TARGET_LEFT_SIDE_OFFSET),
                entry(20, 1 ), //0.0),
                entry(21, 1 ), //0.0),
                entry(24, 3 ), //TURRET_TARGET_RIGHT_SIDE_OFFSET),
                entry(25, 2 ), //TURRET_TARGET_LEFT_SIDE_OFFSET),
                entry(26, 1 ), //0.0),
                entry(27, 2 ) //TURRET_TARGET_LEFT_SIDE_OFFSET))
            ));


        // NOTE: All translations are based on Blue Origin
        public static final Translation2d BLUE_SCORING_LOCATION =
            new Translation2d(
                Meter.of( Units.inchesToMeters(182.11) ),
                Meter.of( Units.inchesToMeters(158.84) ));


        public static final Translation2d RED_SCORING_LOCATION =
            new Translation2d(
                Meter.of( Units.inchesToMeters(651.22 - 182.11) ),
                Meter.of( Units.inchesToMeters(158.84) ));


        // ~34in difference between poles, Perspective based blue origin
        public static final Translation2d RED_CLIMB_NORTH_POLE =
            new Translation2d(
                Meter.of( Units.inchesToMeters(651.22-40.0) ),
                Meter.of( Units.inchesToMeters(187.22)));

        public static final Translation2d RED_CLIMB_SOUTH_POLE =
            new Translation2d(
                Meter.of( Units.inchesToMeters(651.22-40.0) ),
                Meter.of( Units.inchesToMeters(153.22) ));


        public static final Translation2d BLUE_CLIMB_NORTH_POLE =
            new Translation2d(
                Meter.of( Units.inchesToMeters(40.0) ),
                Meter.of( Units.inchesToMeters(187.22) ));

        public static final Translation2d BLUE_CLIMB_SOUTH_POLE =
            new Translation2d(
                Meter.of( Units.inchesToMeters(40.0) ),
                Meter.of( Units.inchesToMeters(153.22) ));

        public static final double ROBOT_X_CLIMBING_OFFSET = 0.0; // TODO: How far does it need to be in either direction to be lined up
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


    public static final class NeoMotorConstants {
        // NEO (REGULAR)
        // SMART CURRENT LIMIT (50A - 60A)
        public static final int SMART_CURRENT_LIMIT_REGULAR = 50;
        public static final double NEO_REG_FREE_SPEED = 5676;
        public static final double NEO_REG_RATING_KV = 473.0; // kv 473

        // NEO 550
        // SMART CURRENT LIMIT (20A - 40A)
        public static final int SMART_CURRENT_LIMIT_550 = 30;
        public static final double NEO_550_FREE_SPEED = 11000;
        public static final double NEO_550_RATING_KV = 915.0; // Neo Vortex KV rating under no load

        // NEO VORTEX
        // SMART CURRENT LIMIT 80A
        public static final int SMART_CURRENT_LIMIT_VORTEX = 80;
        public static final double NEO_VORTEX_FREE_SPEED = 6784;
        public static final double NEO_VORTEX_RATING_KV = 560.0; // Neo Vortex KV rating under no load
    }


    public static class LimelightConstants {
        // Camera names
        public static final String LIMELIGHT_TURRET = "limelight-turret";
        public static final String LIMELIGHT_ROBOT = "limelight-climb";

        // StdDevs for pose estimation trust levels
        public static final double LIMELIGHT_X_STD_DEVS = 0.5;
        public static final double LIMELIGHT_Y_STD_DEVS = 0.5;
        public static final double LIMELIGHT_HEADING_STD_DEVS = 9999999; // StdDevs (x, y, heading)

        public static final double[] LIMELIGHT_ROBOT_CAMERA_POSITION = {
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0
            };
    }


    public static final class TurretConstants {
        // NEO 550
        public static final SparkBaseConfig.IdleMode ROTATE_IDLE_MODE = SparkBaseConfig.IdleMode.kCoast;
        public static final SparkBaseConfig.IdleMode SHOOTER_IDLE_MODE = SparkBaseConfig.IdleMode.kCoast;
        public static final SparkBaseConfig.IdleMode HOOD_IDLE_MODE = SparkBaseConfig.IdleMode.kCoast;

        public static final int ROTATE_MOTOR_ID = 38;
        public static final int SHOOTER_LEAD_MOTOR_ID = 31;
        public static final int SHOOTER_FOLLOWER_MOTOR_ID = 32;
        public static final int HOOD_MOTOR_ID = 22;

        //LimitSwitch DIO Ports
        public static final int HOOD_LIMIT_SWITCH = 0;
        public static final int TURRET_LEFT_LIMIT_SWITCH = 2;
        public static final int TURRET_RIGHT_LIMIT_SWITCH = 3;

        // Max rotation
        public static final double TURRET_MAX_ROTATION = 100.0; // Degrees
        public static final double TURRET_MIN_ROTATION = -100.0;
        public static final double TURRET_ROTATION_DEAD_ZONE = Math.max( 0.0,
            360 - (Math.abs(TURRET_MAX_ROTATION) + Math.abs(TURRET_MIN_ROTATION)));

        public static final double HOOD_MAX_ROTATION = 35.0; // Degrees
        public static final double HOOD_MIN_ROTATION = 0.0;

        public static final double ROTATE_GEAR_RATIO = ((216.0/57.0) * 20.0); // 216:57 * 20:1
        public static final double SHOOTER_GEAR_RATIO = (1/1.25); // 1:1.25 (increase)
        public static final double HOOD_GEAR_RATIO = 20;

        // PID
        public static final double ROTATE_P = 0.0; // TODO: Update
        public static final double ROTATE_I = 0.0;
        public static final double ROTATE_D = 0.0;

        public static final double SHOOTER_P = 0.00075;
        public static final double SHOOTER_I = 0.0;
        public static final double SHOOTER_D = 0.0;

        public static final double HOOD_P = 0.0; // TODO: Update
        public static final double HOOD_I = 0.0;
        public static final double HOOD_D = 0.0;

        // FeedForward // TODO: update

        public static final double ROTATE_KS = 0.51;
        public static final double SHOOTER_KS = 0.185;
        public static final double HOOD_KS = 0; // TODO: Update

        public static final double ROTATE_KV = 0.0; // TODO: Update
        public static final double SHOOTER_KV = 0.001425;
        public static final double HOOD_KV = 0.0; // TODO: Update

        // Conversion factors and expected measured limits
        public static final double ROTATE_POSITION_CONVERSION_FACTOR = (1 / ROTATE_GEAR_RATIO) * 360; // Convert to degrees
        public static final double SHOOTER_POSITION_CONVERSION_FACTOR = ( 1/ SHOOTER_GEAR_RATIO );
        public static final double HOOD_POSITION_CONVERSION_FACTOR = (1 / HOOD_GEAR_RATIO) * 360;

        public static final double ROTATE_VELOCITY_CONVERSION_FACTOR = 1.0;
        public static final double SHOOTER_VELOCITY_CONVERSION_FACTOR = 1.0;
        public static final double HOOD_VELOCITY_CONVERSION_FACTOR = 1.0;

        public static final double ROTATE_MIN_OUTPUT = -0.9;
        public static final double ROTATE_MAX_OUTPUT = 0.9;

        public static final double SHOOTER_MIN_OUTPUT = -0.75;
        public static final double SHOOTER_MAX_OUTPUT = 0.75;

        public static final double HOOD_MIN_OUTPUT = -0.25;
        public static final double HOOD_MAX_OUTPUT = 0.25;

        public static final double ROTATE_ERROR_THRESHOLD = 0.13;
        public static final double SHOOTER_ERROR_THRESHOLD = 0.0;
        public static final double HOOD_ERROR_THRESHOLD = 0.0;

        //  Key: Distance in Feet, Value: {ShooterSpeed, HoodAngle}
        public static final Map<Integer, Double[]> TURRET_LOOKUP_TABLE = new HashMap<>(
            Map.ofEntries(
                    entry(0, new Double[]{ 0.0, 0.0}),
                    entry(1, new Double[]{ 0.0, 0.0}), // expected lower bound
                    entry(2, new Double[]{ 0.0, 0.0}),
                    entry(3, new Double[]{ 0.0, 0.0}),
                    entry(4, new Double[]{ 0.0, 0.0}),
                    entry(5, new Double[]{ 0.0, 0.0}),
                    entry(6, new Double[]{ 0.0, 0.0}),
                    entry(7, new Double[]{ 0.0, 0.0}),
                    entry(8, new Double[]{ 0.0, 0.0}),
                    entry(9, new Double[]{ 0.0, 0.0}),
                    entry(10, new Double[]{ 0.0, 0.0}),
                    entry(11, new Double[]{ 0.0, 0.0}),
                    entry(12, new Double[]{ 0.0, 0.0}),
                    entry(13, new Double[]{ 0.0, 0.0}),
                    entry(14, new Double[]{ 0.0, 0.0}),
                    entry(15, new Double[]{ 0.0, 0.0}),
                    entry(16, new Double[]{ 0.0, 0.0}),
                    entry(17, new Double[]{ 0.0, 0.0}),
                    entry(18, new Double[]{ 0.0, 0.0}),
                    entry(19, new Double[]{ 0.0, 0.0}),
                    entry(20, new Double[]{ 0.0, 0.0}), // expected upper bound
                    entry(21, new Double[]{ 0.0, 0.0})
        ));
    }


    public static final class IntakeConstants {
        // Extension idle modes & Pickup idle modes
        public static final SparkBaseConfig.IdleMode EXTENSION_IDLE_MODE = SparkBaseConfig.IdleMode.kCoast; //TODO: set back to kBrake once we're done testing inverses-SHing
        public static final SparkBaseConfig.IdleMode PICKUP_IDLE_MODE = SparkBaseConfig.IdleMode.kCoast;

        //"intake" refers to the intake rollers
        public static final double INTAKE_SPEED_SLOW = 0.2;
        public static final double INTAKE_SPEED_FAST = 0.6;

             // ID's
        public static final int STARBOARD_EXTENSION_MOTOR_ID = 14;
        public static final int PORT_EXTENSION_MOTOR_ID = 13;//TODO: set values here
        public static final int PICKUP_MOTOR_ID = 21; //TODO: set values here

        // Limit Switches, Extended is when extended, retracted is when retracted, two pairs of switches each, you get it
        public static final int EXTENDED_LSChannel_PORT = 6;
        public static final int EXTENDED_LSChannel_STARBOARD = 9;
        public static final int RETRACTED_LSChannel_PORT = 7;
        public static final int RETRACTED_LSChannel_STARBOARD = 8;

        public static final double EXTENSION_GEAR_RATIO = 1.0;
        public static final double PICKUP_GEAR_RATIO = 1.0;

        // Extension Speeds & Pickup Speeds (Closed-Loop)
        public static final double EXTENSION_FULL_FREE_SPEED = 11000; //
        public static final double PICKUP_FAST_SPEED_SETPOINT = 2500; //in rpm, about half of max rpm for a neo brushless
        public static final double PICKUP_SLOW_SPEED_SETPOINT = 500; //TODO: change to an actual number

        public static final double PICKUP_SLOW_SPEED = 0.10; //TODO: change to an actual number, also what is this even used for

        // Extension PID
        public static final double EXTENSION_P = 0.0; //TODO: set PID values
        public static final double EXTENSION_I = 0.0;
        public static final double EXTENSION_D = 0.0;

        public static final double PICKUP_P = 0.0;
        public static final double PICKUP_I = 0.0;
        public static final double PICKUP_D = 0.0;

        // Extension FeedForward
        public static final double EXTENSION_KS = 0.0;
        public static final double EXTENSION_KV = 0.0;

        public static final double PICKUP_KS = 0.0;
        public static final double PICKUP_KV = 0.0;

        public static final double EXTENSION_POSITION_CONVERSION_FACTOR = 2 * Math.PI * 0.762; // Convert to rev to cm. 0.762 in radius of gear in cm
        public static final double EXTENSION_VELOCITY_CONVERSION_FACTOR = 1.0;
        public static final double EXTENSION_MIN_OUTPUT = -0.75;
        public static final double EXTENSION_MAX_OUTPUT = 0.75;
        public static final double EXTENSION_SLOW_SPEED = 0.10;
        public static final double EXTENSION_FAST_SPEED = 0.30;
        public static final double EXTENSION_MAX_ANGLE = 0.00; //TODO: find angles
        public static final double MIN_ANGLE = 0.00;

        public static final double EXTENSION_ERROR_THRESHOLD = 0.0; // TODO: Update
        public static final double PICKUP_ERROR_THRESHOLD = 0.0;

        // Pickup values
        public static final double PICKUP_POSITION_CONVERSION_FACTOR = 2 * Math.PI * 0.762; // Convert to rev to cm. 0.762 in radius of gear in cm
        public static final double PICKUP_VELOCITY_CONVERSION_FACTOR = 1.0;
        public static final double PICKUP_MIN_OUTPUT = -0.75;
        public static final double PICKUP_MAX_OUTPUT = 0.75;

        //End of Rail values
        public static final double INTAKE_RAIL_END = 31.0; //
        public static final double INTAKE_RAIL_START = 0.0;
    }


    public static final class SpindexerConstants {
        public static final SparkBaseConfig.IdleMode FEED_IDLE_MODE = SparkBaseConfig.IdleMode.kCoast;
        public static final SparkBaseConfig.IdleMode ROTATE_IDLE_MODE = SparkBaseConfig.IdleMode.kCoast;

        //TODO: change current limit to lower?
        public static final int FEED_CURRENT_LIMIT = 60;
        public static final int ROTATE_CURRENT_LIMIT = 50;

        // Spark ID'S
        public static final int ROTATE_MOTOR_ID = 25;
        public static final int FEED_MOTOR_ID = 26;

        // Feed gear ratios & Rotate gear ratios TODO: UPDATE
        public static final double FEED_GEAR_RATIO = 1; // 1:1
        public static final double ROTATE_GEAR_RATIO = 4.0 * (57.0 / 22.0); // 4:1 * (57:22)

        // PIDs
        public static final double FEED_P = 0.0;
        public static final double FEED_I = 0.0;
        public static final double FEED_D = 0.0;

        public static final double ROTATE_P = 0.0;
        public static final double ROTATE_I = 0.0;
        public static final double ROTATE_D = 0.0;

        // FeedForward // TODO: UPDATE
        public static final double FEED_KV = 0;
        public static final double ROTATE_KV = 0;
        public static final double FEED_KS = 0.185;
        public static final double ROTATE_KS = 0.185;

        public static final double FEED_POSITION_CONVERSION_FACTOR = 1.0;
        public static final double ROTATE_POSITION_CONVERSION_FACTOR = 1.0;

        public static final double FEED_VELOCITY_CONVERSION_FACTOR = 1.0; // RPM
        public static final double ROTATE_VELOCITY_CONVERSION_FACTOR = 1.0; // RPM

        public static final double FEED_MIN_OUTPUT = -0.75;
        public static final double FEED_MAX_OUTPUT = 0.75;

        public static final double FEED_MAX_SPEED = 0.7; // set between -1 to 1
        public static final double FEED_SLOW_SPEED = 0.5;

        public static final double ROTATE_MIN_OUTPUT = -0.75;
        public static final double ROTATE_MAX_OUTPUT = 0.75;

        public static final double ROTATE_SLOW_SPEED = 0.01;
        public static final double ROTATE_MAX_SPEED = 0.75; // set between -1 to 1

        public static final double ROTATE_ERROR_THRESHOLD = 0.0;
        public static final double FEED_ERROR_THRESHOLD = 0.0;
    }


    public static final class ClimbConstants {
        public static final SparkBaseConfig.IdleMode IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;

        // IDS
        public static final int CLIMB_MOTOR_ID = 35;
        public static final int CLIMB_LSChannel = 5;

        public static final double CLIMB_GEAR_RATIO = 20; // 20:1

        public static final double CLIMB_POSITION_CONVERSION_FACTOR = (1 / CLIMB_GEAR_RATIO) * 360; // Convert to degrees
        public static final double CLIMB_VELOCITY_CONVERSION_FACTOR = 1.0;

        public static final double CLIMB_P = 0.0; //TODO: set PID values
        public static final double CLIMB_I = 0.0;
        public static final double CLIMB_D = 0.0;

        // NEO REG
        public static final double CLIMB_KS = 0.0;
        public static final double CLIMB_KV = 0.0;

        public static final double CLIMB_MIN_OUTPUT = -0.75;
        public static final double CLIMB_MAX_OUTPUT = 0.75;

        public static final double CLIMB_ERROR_THRESHOLD = 0.0;

        // TODO: Update with real world values
        public static final double CLIMB_MIN_ANGLE = 0.0;
        public static final double CLIMB_MAX_ANGLE = 0.0;
        public static final double CLIMB_DEPLOY_ANGLE = 6600.0; //no idea what units these are in. AHS kids, ask mr,joo about the shing rule
        public static final double CLIMB_THRESHOLD_ANGLE = 5700.0;
        public static final double CLIMB_UP_ANGLE = 0.0; // Placeholder, please change.
        public static final double CLIMB_DOWN_ANGLE = 0.0; // Placeholder, please change.
        public static final double CLIMB_L1_ANGLE = 0.0;


        public static final double CLIMB_FAST_SPEED = 0.8;
        public static final double CLIMB_MAX_SPEED = 1.0;
        public static final double CLIMB_SLOW_SPEED = 0.3;
    }
}
