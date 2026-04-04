// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.commands.intakecommands.ExtendIntakeCommand;
import frc.robot.commands.intakecommands.RetractIntakeCommand;
import frc.robot.commands.turretcommands.AutoTurretTargeting;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.*;
import frc.robot.motors.Components;
import java.io.File;

import frc.robot.vision.LimelightRunner;
import swervelib.SwerveInputStream;
import frc.robot.commands.climbCommand.*;
import frc.robot.Constants.IntakeConstants;

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
    private final CommandStadiaController driverStadia = new CommandStadiaController(5);
    private final CommandJoystick joystickDriver = new CommandJoystick(0); //set port 0 for stadia/joystick, whichever is being used
    private final CommandJoystick joystickOperator = new CommandJoystick(1);

    private final Components motorComponents = Components.getInstance();

    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve/maxSwerve"));

    // Establish a Sendable Chooser that will be able to be sent to the
    // SmartDashboard, allowing selection of desired auto
    private final SendableChooser<Command> autoChooser;


// TODO: Test and uncomment subsystems

    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(
            motorComponents.getIntakeExtensionStarboardMotor(),
            motorComponents.getIntakeExtensionPortMotor(),
            motorComponents.getIntakePickupMotor(),
//            motorComponents.getIntakeExtensionMotorPair(),
            new DigitalInput(IntakeConstants.EXTENDED_LSChannel_STARBOARD),
            new DigitalInput(IntakeConstants.EXTENDED_LSChannel_PORT),
            new DigitalInput(IntakeConstants.RETRACTED_LSChannel_STARBOARD),
            new DigitalInput(IntakeConstants.RETRACTED_LSChannel_PORT)
    );


    private final SpindexerSubsystem spindexerSubsystem = new SpindexerSubsystem(
            motorComponents.getSpindexerRotateMotor(),
            motorComponents.getSpindexerFeedMotor()
    );


    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem(
            motorComponents.getClimbMotor(),
            new DigitalInput(Constants.ClimbConstants.CLIMB_LSChannel)
    );


    private final TurretSubsystem turretSubsystem = new TurretSubsystem(
            motorComponents.getTurretShooterMotor(),
            motorComponents.getTurretRotateMotor(),
            motorComponents.getTurretHoodMotor(),
            new DigitalInput(Constants.TurretConstants.HOOD_LIMIT_SWITCH),
            new DigitalInput(Constants.TurretConstants.TURRET_LEFT_LIMIT_SWITCH),
            new DigitalInput(Constants.TurretConstants.TURRET_RIGHT_LIMIT_SWITCH)
    );


    //Intake Commands
    ExtendIntakeCommand extendIntakeCommand = new ExtendIntakeCommand(intakeSubsystem);
    RetractIntakeCommand retractIntakeCommand = new RetractIntakeCommand(intakeSubsystem);

    //Climb Commands
    DeployClimbCommand deployClimbCommand = new DeployClimbCommand(climbSubsystem, Constants.ClimbConstants.CLIMB_SLOW_SPEED);
    RetractClimbCommand retractClimbCommand = new RetractClimbCommand(climbSubsystem, Constants.ClimbConstants.CLIMB_SLOW_SPEED);

    AutoTurretTargeting simpleTurretTracking = new AutoTurretTargeting(turretSubsystem);

    // NOTE:  Coords are odd for Joysticks: https://docs.wpilib.org/en/stable/docs/software/basic-programming/joystick.html
    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled
     * by angular velocity.
     */
    SwerveInputStream driveAngularVelocityBlueJoystick = SwerveInputStream.of(
                    drivebase.getSwerveDrive(),
            () -> attenuated( joystickDriver.getY(), 2, 1.0 ) * 1,
            () -> attenuated( joystickDriver.getX(), 2, 1.0 ) * 1)
            .withControllerRotationAxis(
                    () -> attenuated( joystickDriver.getTwist(), 2, 0.75 ) * 1)
            .deadband(OperatorConstants.DEADBAND)
            .allianceRelativeControl(true);

    SwerveInputStream driveAngularVelocitySlowBlueJoystick = SwerveInputStream.of(
            drivebase.getSwerveDrive(),
            () -> attenuated( joystickDriver.getY(), 2, 0.25 ) * 1,
            () -> attenuated( joystickDriver.getX(), 2, 0.25 ) * 1)
        .withControllerRotationAxis(
            () -> attenuated( joystickDriver.getTwist(), 2, 0.75 ) * 1)//scale originally 0.5
        .deadband(OperatorConstants.DEADBAND)
        .allianceRelativeControl(true);


//    // Copy of above but with x, y flipped for red side alliance
//    SwerveInputStream driveAngularVelocityRedJoystick = SwerveInputStream.of(
//                    drivebase.getSwerveDrive(),
//                    () -> attenuated( joystickDriver.getY(), 2, 1.0 ) * 1,
//                    () -> attenuated( joystickDriver.getX(), 2, 1.0 ) * 1)
//            .withControllerRotationAxis(
//                    () -> attenuated( joystickDriver.getTwist(), 3, 0.75 ) * -1)
//            .deadband(OperatorConstants.DEADBAND)
//            .allianceRelativeControl(true);
//
//    // Copy of above but with x, y flipped for red side alliance
//    SwerveInputStream driveAngularVelocitySlowRedJoystick = SwerveInputStream.of(
//                    drivebase.getSwerveDrive(),
//                    () -> attenuated( joystickDriver.getY(), 2, 0.5 ) * 1,
//                    () -> attenuated( joystickDriver.getX(), 2, 0.5 ) * 1)
//            .withControllerRotationAxis(
//                    () -> attenuated( joystickDriver.getTwist(), 3, 0.375 ) * -1)
//            .deadband(OperatorConstants.DEADBAND)
//            .allianceRelativeControl(true);


    SwerveInputStream driveRobotRelativeSlowJoystick = SwerveInputStream.of(
                    drivebase.getSwerveDrive(),
                    () -> attenuated( joystickDriver.getY(), 2, 0.25 ) * 1,
                    () -> attenuated( joystickDriver.getX(), 2, 0.25 ) * 1)
            .withControllerRotationAxis(
                    () -> attenuated( joystickDriver.getTwist(), 3, 0.25 ) * -1)
            .deadband(OperatorConstants.DEADBAND)
            .robotRelative(true);


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
        NamedCommands.registerCommand("climbArmUp",Commands.runOnce(() ->
                climbSubsystem.simpleClimbDeploy(1.0), climbSubsystem).repeatedly());
        NamedCommands.registerCommand("climbArmDown",Commands.runOnce(() ->
                climbSubsystem.simpleClimbMoveDown(-1.0), climbSubsystem).repeatedly());


        //Have the autoChooser pull in all PathPlanner autos as options
        autoChooser = AutoBuilder.buildAutoChooser();

        // Set the default auto (do nothing)
        autoChooser.setDefaultOption("Do Nothing", Commands.runOnce(drivebase::zeroGyroWithAlliance)
                .andThen(Commands.none()));

        // Add a simple auto option to have the robot drive forward for 1 second then
        // stop
        autoChooser.addOption("Drive Forward",
                Commands.runOnce(drivebase::zeroGyroWithAlliance).withTimeout(.2)
                .andThen(drivebase.driveForward().withTimeout(1)));

        autoChooser.addOption("Drive Backward",
                Commands.runOnce(drivebase::zeroGyroWithAlliance).withTimeout(.2)
                        .andThen(drivebase.driveBackward().withTimeout(1)));

        autoChooser.addOption(
                "Back Up and Shoot",
                Commands.runOnce(drivebase::zeroGyroWithAlliance).withTimeout(.2)
                        .andThen( turretSubsystem.setShooterMotor(3000).withTimeout(0.25))
                        .andThen(drivebase.driveBackward().withTimeout(1))
                        .andThen(Commands.run(()->turretSubsystem.moveHoodUp(5,0.1)).withTimeout(0.6))
                        //.andThen(Commands.runOnce(() -> turretSubsystem.updateTurretRotation(20.0), turretSubsystem)).withTimeout(0.25)
                        .andThen(Commands.runOnce(spindexerSubsystem::feed, spindexerSubsystem).withTimeout(1.0))
                        //.andThen(Commands.runOnce(() -> turretSubsystem.updateTurretRotation(-20.0)).withTimeout(0.25))

//                        .andThen(Commands.runOnce(intakeSubsystem::extendIntake).repeatedly().withTimeout(1))
//                        .andThen(Commands.runOnce(intakeSubsystem::retractIntake).repeatedly().withTimeout(1))
//                        .andThen(Commands.runOnce(spindexerSubsystem::feed).repeatedly().withTimeout(3))
                // .andThen(Commands.runOnce(()-> turretSubsystem.moveHoodUp())))
                        //.alongWith()-
                //                        .andThen().withTimeout(1.0)
//                        .andThen().withTimeout(1.0)
//                        .andThen().withTimeout(1.0)
        );

        // Put the autoChooser on the SmartDashboard
        SmartDashboard.putData("Auto Chooser", autoChooser);

        if (autoChooser.getSelected() == null) {
            RobotModeTriggers.autonomous().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));
        }

        // Update Alliance Relevant info
        updateDriverAllianceInfo();
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

        Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
        Command driveFieldOrientedAngularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);

        Command driveFieldOrientedStadia = drivebase.driveFieldOriented(driveStadia);

        Command driveFieldOrientedPS5 = drivebase.driveFieldOriented(driveFieldPS5);
        Command driveFieldHeadingPS5 = drivebase.driveFieldOriented(driveHeadingAxisPS5);

        Command driveFieldOrientedBlueAlliance = drivebase.driveFieldOriented(driveAngularVelocityBlueJoystick);
        Command driveFieldOrientedBlueAllianceSlow = drivebase.driveFieldOriented(driveAngularVelocitySlowBlueJoystick);


        if (RobotBase.isSimulation()) {
            // drivebase.setDefaultCommand(driveFieldOrientedPS5);
            // drivebase.setDefaultCommand(driveFieldHeadingPS5);

            drivebase.setDefaultCommand(driveFieldOrientedBlueAlliance);
            joystickDriver.button(11).whileTrue(driveFieldOrientedBlueAllianceSlow);
            joystickDriver.button(5).whileTrue(drivebase.centerModulesCommand());
            joystickDriver.button(6).whileTrue(Commands.runOnce(drivebase::lock));

        } else {
            // drivebase.setDefaultCommand(driveFieldOrientedAngularVelocityJoystick);
            // updateDriverAllianceControls(); // TODO: Confirm one setup works for both sides of the field after field Zero
            drivebase.setDefaultCommand(driveFieldOrientedBlueAlliance);
            joystickDriver.button(11).whileTrue(driveFieldOrientedBlueAllianceSlow);


        }

        if (Robot.isSimulation()) {

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
            //            joystickOperator.button(10)
//                    .onTrue(Commands.runOnce(() -> drivebase.resetOdometry(startPose)));
//
//            joystickOperator.button(11).whileTrue(drivebase.driveToPose(targetPose));

            //joystickOperator.trigger().whileTrue(Commands.runOnce(spindexerSubsystem::feed, spindexerSubsystem).repeatedly());

        } else {
           //Teleop Command Keybinds

            // TODO: Test align to target on field, physically align the robot to ideal position and note it here
            // Ideally as an x offset that would work for +- depending on side of approach
            // RED N, RED S, BLUE N, BLUE S
            Pose2d startPose = new Pose2d(new Translation2d(3.580, 4.179),
                    Rotation2d.fromDegrees(180));
            Pose2d targetPose = new Pose2d(new Translation2d(2.666, 4.179),
                    Rotation2d.fromDegrees(180));

        //         PLACE ALL TELEOP KEYBINDS HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

            turretSubsystem.setDefaultCommand(Commands.runOnce(turretSubsystem::returnTurretToZero, turretSubsystem));


           //DRIVER:
           joystickDriver.trigger()
                   .whileTrue(spindexerSubsystem.rotateMotors())
                   .whileFalse(spindexerSubsystem.stopFeed());
            joystickDriver.button(2)
                    .whileTrue(intakeSubsystem.outake(0.2))
                    .whileFalse(intakeSubsystem.autoSetPickupSpeed());  //   TODO:  create command to set pickup speed reverse, has priority over auto set
           //TODO: create command to rotate turret maually for buttons 3,4,and5
           joystickDriver.button(6)
                   .whileTrue(Commands.runOnce(() -> turretSubsystem.moveHoodDown(0.0),turretSubsystem).repeatedly());
           joystickDriver.button(7)
                    .whileTrue(Commands.runOnce(
                            ()->climbSubsystem.simpleClimbDeploy(Constants.ClimbConstants.CLIMB_MAX_SPEED), climbSubsystem).repeatedly())
                    .onFalse(Commands.runOnce(climbSubsystem::climbStop, climbSubsystem));
           joystickDriver.button(8)
                    .whileTrue(Commands.runOnce(
                            ()->climbSubsystem.simpleClimbMoveDown(Constants.ClimbConstants.CLIMB_MAX_SPEED), climbSubsystem).repeatedly())
                    .onFalse(Commands.runOnce(climbSubsystem::climbStop, climbSubsystem));
           //joystickDriver.button(9).whileTrue(drivebase.centerModulesCommand());TODO: might be useful for testing?
           joystickDriver.button(9).whileTrue(Commands.runOnce(drivebase::lock));
           joystickDriver.button(12).onTrue((Commands.runOnce(drivebase::zeroGyroWithAlliance)));




            //OPERATOR:
            joystickOperator.trigger()
                    .whileTrue(simpleTurretTracking)
                    .onFalse(Commands.run(
                            () -> turretSubsystem.moveHoodDown(0.0),turretSubsystem).withTimeout(0.5));

            joystickOperator.button(3)
                .whileTrue(turretSubsystem.setShooterMotor(0.0));
            joystickOperator.button(4)
                    .whileTrue(intakeSubsystem.retractIntake())
                    .whileFalse(Commands.runOnce(intakeSubsystem::stopTranslation, intakeSubsystem));
            joystickOperator.button(5)
                    .whileTrue(Commands.runOnce(() -> turretSubsystem.moveHoodDown(0.0),turretSubsystem).repeatedly())
                    .onFalse(Commands.runOnce(turretSubsystem::stopHoodMotor, turretSubsystem));
            joystickOperator.button(6)
                    .whileTrue(intakeSubsystem.extendIntake())
                    .whileFalse(Commands.runOnce(intakeSubsystem::stopTranslation, intakeSubsystem));

            joystickOperator.button(7)
                    .whileTrue(Commands.runOnce(() -> turretSubsystem.moveHoodUp(35.0,0.4),turretSubsystem).repeatedly())
                    .onFalse(Commands.runOnce(turretSubsystem::stopHoodMotor, turretSubsystem));
            joystickOperator.button(9)
                    .whileTrue(Commands.runOnce(() -> turretSubsystem.moveHoodUp(25.0,0.225),turretSubsystem).repeatedly())
                    .onFalse(Commands.runOnce(turretSubsystem::stopHoodMotor, turretSubsystem));
            joystickOperator.button(11)
                    .whileTrue(Commands.runOnce(() -> turretSubsystem.moveHoodUp(15.0,0.18),turretSubsystem).repeatedly())
                    .onFalse(Commands.runOnce(turretSubsystem::stopHoodMotor, turretSubsystem));

            joystickOperator.button(8)
                            .whileTrue(turretSubsystem.setShooterMotor(4500.0));
            joystickOperator.button(10)
                            .whileTrue(turretSubsystem.setShooterMotor(3750.0));
            joystickOperator.button(12)
                            .whileTrue(turretSubsystem.setShooterMotor(3000.0));






            joystickDriver.button(10).whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

            // TODO: Shuffle Intake
            // joystickOperator.button( <> )

            // TODO: set default command to go to 0
            // turretSubsystem.setDefaultCommand(Commands.runOnce(turretSubsystem::returnTurretToZero).repeatedly());

            // TODO: passing turret button ( Aim towards wall using gyro)

            // TODO: Passing turret button ( Aim towards coordinates )

            // TODO: scoring turret button (Simple align to april tag)

            // TODO: scoring turret button (Align with update pose offset) TEST TO CONFIRM, this might act strange if we are not careful

            // TODO: Shooting aka spindexer and motor feed balls if shooter wheel is spinning (we should also force the shooter wheel to be kcoast by default anyways)



//            joystickOperator.button(2)
//                    .whileTrue(intakeSubsystem.stopPickupMotor());


//            joystickOperator.button(5)
//                    .whileTrue(intakeSubsystem.setSlowPickup(IntakeConstants.INTAKE_SPEED_SLOW))
//                    .whileFalse(intakeSubsystem.stopPickupMotor());




//
//            joystickOperator.button(7)
//                    .whileTrue(intakeSubsystem.shuffleIntakeCommand())
//                    .whileFalse(Commands.runOnce(intakeSubsystem::stopTranslation, intakeSubsystem));





            // TODO: Test setting to specific hood angle
            // joystickOperator.button(12).onTrue(Commands.runOnce(() -> turretSubsystem.updateHoodAngle(20)));

            // TODO: Orientation will depend on side it is approached from, give translation constant and set orientation here
//            joystickDriver.button(9).whileTrue(drivebase.driveToPose(
//                new Pose2d(
//                    Constants.StructureConstants.RED_CLIMB_NORTH_POLE.getX()+Constants.StructureConstants.ROBOT_X_CLIMBING_OFFSET,
//                    Constants.StructureConstants.RED_CLIMB_NORTH_POLE.getY(),
//                    Rotation2d.fromDegrees(0))));

            //TODO: Uncomment for drive team after subsystem testing
            //Intake Command keybind
//            joystickDriver.button(5).onTrue(extendIntakeCommand);
//            joystickDriver.button(3).onTrue(retractIntakeCommand);
//            joystickOperator.button(2).onTrue(intakeSubsystem.setSlowPickup(IntakeConstants.PICKUP_SLOW_SPEED));
//            joystickOperator.button(7).onTrue(intakeSubsystem.stopPickupMotor());

            //Climb Command keybinds

//            driverStadia.leftTrigger().whileTrue(driveFieldOrientedAngularVelocityStadia);

            driverStadia.leftBumper().onTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
            driverStadia.rightBumper().onTrue((Commands.runOnce(drivebase::zeroGyro)));
            // driverStadia.a().whileTrue(drivebase.driveToPose(targetPose));

            driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
            driverXbox.start().whileTrue(Commands.none());
            driverXbox.back().whileTrue(Commands.none());
            driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
            driverXbox.rightBumper().onTrue(Commands.none());

        }
    }

    //TODO: Finish at field
    public void updateDriverAllianceInfo(){
        var alliance = DriverStation.getAlliance();
        LimelightRunner limelightRunner = LimelightRunner.getInstance();
        String turret = Constants.LimelightConstants.LIMELIGHT_TURRET;

        boolean isRedAlliance = (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red);

        // limelightRunner.turretPipelineSetup( isRedAlliance );
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to rotateMotors in autonomous
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
