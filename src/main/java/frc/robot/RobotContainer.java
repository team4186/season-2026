// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.motors.Components;
import java.io.File;

import frc.robot.vision.LimelightHelpers;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driverXbox = new CommandXboxController(3);
    private final CommandPS5Controller driverPS5 = new CommandPS5Controller(4);
    private final CommandStadiaController driverStadia = new CommandStadiaController(0);
    private final CommandJoystick joystickDriver = new CommandJoystick(1); //set port 0 for stadia/joystick, whichever is being used

    private final Components motorComponents = Components.getInstance();

    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve/maxSwerve"));

    // Establish a Sendable Chooser that will be able to be sent to the
    // SmartDashboard, allowing selection of desired auto
    private final SendableChooser<Command> autoChooser;

//    private final IntakeSubsystem intake = new IntakeSubsystem(
//            motorComponents.getIntakePickupMotor(),
//            motorComponents.getIntakeDeployMotor(),
//            new DigitalInput(Constants.IntakeConstants.INTAKE_EXTENDED_LSChannel1),
//            new DigitalInput(Constants.IntakeConstants.INTAKE_EXTENDED_LSChannel2),
//            new DigitalInput(Constants.IntakeConstants.INTAKE_RETRACTED_LSChannel1),
//            new DigitalInput(Constants.IntakeConstants.INTAKE_RETRACTED_LSChannel2)
//
//    );

//    private final SpindexerSubsystem spindexer = new SpindexerSubsystem(
//            motorComponents.getSpindexerRotateMotor(),
//            motorComponents.getSpindexerFeedMotor()
//    );



    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled
     * by angular velocity.
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> driverXbox.getLeftY() * -1,
            () -> driverXbox.getLeftX() * -1)
            .withControllerRotationAxis(driverXbox::getRightX)
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

//    SwerveInputStream driveAngularVelocityPS5 = SwerveInputStream.of(drivebase.getSwerveDrive(),
//                    () -> driverPS5.getLeftY() * -1,
//                    () -> driverPS5.getLeftX() * -1)
//            .withControllerRotationAxis(
//                    () -> driverPS5.getRawAxis(3))
//            .deadband(OperatorConstants.DEADBAND)
//            .scaleTranslation(0.8)
//            .allianceRelativeControl(true);

    SwerveInputStream driveAngularVelocityStadia = SwerveInputStream.of(drivebase.getSwerveDrive(),
                    () -> attenuated( driverStadia.getLeftY() * -1, 2, 0.6),
                    () -> attenuated( driverStadia.getLeftX(), 2, 0.60) * 1)
            .withControllerRotationAxis(
                    () -> attenuated(driverStadia.getRawAxis(3), 2, 0.6))
                    //driverStadia::getRightX)
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

//    SwerveInputStream driveDirectAnglePS5 = driveAngularVelocityPS5.copy()
//            .withControllerHeadingAxis(
//                    () -> driverPS5.getRawAxis(3),
//                    () -> driverPS5.getRawAxis(4))
//            .headingWhile(true);0

    SwerveInputStream driveDirectAngleStadia = driveAngularVelocityStadia.copy()
            .withControllerHeadingAxis(
                    () -> attenuated( driverStadia.getRawAxis(3), 2, 0.60),
                    () -> attenuated( driverStadia.getRawAxis(4), 2, 0.60) * -1)
            .headingWhile(true);

    /**
     * Clone's the angular velocity input stream and converts it to a fieldRelative
     * input stream.
     */
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
            driverXbox::getRightY)
            .headingWhile(true);

    /**
     * Clone's the angular velocity input stream and converts it to a robotRelative
     * input stream.
     */
    SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
            .allianceRelativeControl(false);

    SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> -driverXbox.getLeftY(),
            () -> -driverXbox.getLeftX())
            .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                    2))
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

    // Derive the heading axis with math!
    SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
            .withControllerHeadingAxis(() -> Math.sin(
                    driverXbox.getRawAxis(
                            2) *
                            Math.PI)
                    *
                    (Math.PI *
                            2),
                    () -> Math.cos(
                            driverXbox.getRawAxis(
                                    2) *
                                    Math.PI)
                            *
                            (Math.PI *
                                    2))
            .headingWhile(true)
            .translationHeadingOffset(true)
            .translationHeadingOffset(Rotation2d.fromDegrees(
                    0));

    SwerveInputStream driveStadia = SwerveInputStream.of(
                    drivebase.getSwerveDrive(),
                    () -> attenuated( driverStadia.getLeftY(), 2, 1.0 ) * -1,
                    () -> attenuated( driverStadia.getLeftX(), 2, 1.0 ) * -1)
            .withControllerRotationAxis(
                     () -> driverStadia.getRawAxis(3))
                    //driverStadia::getRightX)
            // () -> attenuated( joystickDriver.getTwist(), 3, 0.75 ) * 1)
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.4)
            .allianceRelativeControl(true);

    SwerveInputStream driveStadiaHeadingAxis = driveStadia.copy().withControllerHeadingAxis(
                    driverStadia::getRightX,
                    driverStadia::getRightY)
            .headingWhile(true);

    SwerveInputStream driveAngularVelocityJoystick = SwerveInputStream.of(
                    drivebase.getSwerveDrive(),
                    () -> attenuated( joystickDriver.getY(), 2, 1.0 ) * -1,
                    () -> attenuated( joystickDriver.getX(), 2, 1.0 ) * -1)
            .withControllerRotationAxis(
                    () -> attenuated( joystickDriver.getTwist(), 3, 0.75 ) * 1)
            .deadband(OperatorConstants.DEADBAND)
            .allianceRelativeControl(true);

    SwerveInputStream driveFieldPS5 = SwerveInputStream.of(
            drivebase.getSwerveDrive(),
            () -> attenuated( driverPS5.getLeftY(), 2, 1.0 ) * -1,
            () -> attenuated( driverPS5.getLeftX(), 2, 1.0 ) * -1)
        .withControllerRotationAxis(
            driverPS5::getRightX)
        .deadband(OperatorConstants.DEADBAND)
        .allianceRelativeControl(true);

    SwerveInputStream driveHeadingAxisPS5 = driveFieldPS5.copy().withControllerHeadingAxis(
            driverPS5::getRightX,
            driverPS5::getRightY)
        .headingWhile(true);


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);

        // Create the NamedCommands that will be used in PathPlanner
        NamedCommands.registerCommand("test", Commands.print("I EXIST"));

        //Have the autoChooser pull in all PathPlanner autos as options
        autoChooser = AutoBuilder.buildAutoChooser();

        // Set the default auto (do nothing)
        autoChooser.setDefaultOption("Do Nothing", Commands.runOnce(drivebase::zeroGyroWithAlliance)
                .andThen(Commands.none()));

        // Add a simple auto option to have the robot drive forward for 1 second then
        // stop
        autoChooser.addOption("Drive Forward", Commands.runOnce(drivebase::zeroGyroWithAlliance).withTimeout(.2)
                .andThen(drivebase.driveForward().withTimeout(1)));

        // Put the autoChooser on the SmartDashboard
        SmartDashboard.putData("Auto Chooser", autoChooser);

        if (autoChooser.getSelected() == null) {
            RobotModeTriggers.autonomous().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));
        }
    }


    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary predicate, or via the
     * named factories in
     * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
     * for
     * {@link CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
     * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
     * Flight joysticks}.
     */
    private void configureBindings() {
        Command driveFieldOrientedAngularVelocityJoystick = drivebase.driveFieldOriented(driveAngularVelocityJoystick);

        Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
        Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity); // SIM
        Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
        Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);

        Command driveFieldOrientedAngularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
        Command driveFieldOrientedAngularVelocityStadia = drivebase.driveFieldOriented(driveAngularVelocityStadia);
        Command driveFieldOrientedStadia = drivebase.driveFieldOriented(driveStadia);

        Command driveFieldOrientedAngularVelocityStadiaAngle = drivebase.driveFieldOriented(driveDirectAngleStadia);

        Command driveFieldOrientedPS5 = drivebase.driveFieldOriented(driveFieldPS5);
        Command driveFieldHeadingPS5 = drivebase.driveFieldOriented(driveHeadingAxisPS5);


        if (RobotBase.isSimulation()) {
            // drivebase.setDefaultCommand(driveFieldOrientedPS5);
            drivebase.setDefaultCommand(driveFieldHeadingPS5);
        } else {
            // drivebase.setDefaultCommand(driveFieldOrientedAngularVelocityJoystick);
            drivebase.setDefaultCommand(driveFieldOrientedStadia);
        }

        if (Robot.isSimulation()) {
            Pose2d target = new Pose2d(new Translation2d(1, 4),
                    Rotation2d.fromDegrees(90));
            // drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
            driveDirectAngleKeyboard.driveToPose(() -> target,
                    new ProfiledPIDController(5,
                            0,
                            0,
                            new Constraints(5, 2)),
                    new ProfiledPIDController(5,
                            0,
                            0,
                            new Constraints(Units.degreesToRadians(360),
                                    Units.degreesToRadians(180))));
            driverXbox.start()
                    .onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
            driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
            driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                    () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

            // Testing PS5 Controls in Sim, both work as intended
            driverPS5.L1().whileTrue( driveFieldOrientedPS5 );
            driverPS5.R1().whileTrue( driveFieldHeadingPS5 );

            // TODO: Test with stadia controller on real robot, manual set pose might be helpful for testing?
            // Create a target pose with destination, hold button to drive to pose
            Pose2d targetPose = new Pose2d(new Translation2d(15, 4),
                    Rotation2d.fromDegrees(180));
            driverPS5.cross().whileTrue(drivebase.driveToPose(targetPose));

        }

        if (DriverStation.isTest()) {
            driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
            driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
            driverXbox.back().whileTrue(drivebase.centerModulesCommand());
            driverXbox.leftBumper().onTrue(Commands.none());
            driverXbox.rightBumper().onTrue(Commands.none());

        } else {
           //Teleop Command Keybinds

            joystickDriver.button(11).onTrue((Commands.runOnce(drivebase::zeroGyro)));
            joystickDriver.button(12).whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());


            driverStadia.leftTrigger().whileTrue(driveFieldOrientedAngularVelocityStadia);
            Pose2d targetPose = new Pose2d(new Translation2d(15, 4),
                    Rotation2d.fromDegrees(180));

            driverStadia.leftBumper().onTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
            driverStadia.rightBumper().onTrue((Commands.runOnce(drivebase::zeroGyro)));
            driverStadia.a().whileTrue(drivebase.driveToPose(targetPose));

            driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
            driverXbox.start().whileTrue(Commands.none());
            driverXbox.back().whileTrue(Commands.none());
            driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
            driverXbox.rightBumper().onTrue(Commands.none());

        }
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Pass in the selected auto from the SmartDashboard as our desired autnomous
        // commmand
        return autoChooser.getSelected();
    }


    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }


    private double attenuated(double value, int exponent, double scale) {
        double res = scale * Math.pow( Math.abs(value), exponent );
        if ( value < 0 ) { res *= -1; }
        return res;
    }
}
