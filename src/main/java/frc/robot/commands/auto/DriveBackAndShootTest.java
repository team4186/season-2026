package frc.robot.commands.auto;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.vision.LimelightRunner;

public class DriveBackAndShootTest extends Command {


    private final TurretSubsystem turret;
    private final SwerveSubsystem swerve;
    private final SpindexerSubsystem spindexer;
    private final IntakeSubsystem intake;
    private boolean isCommandFinished;

    private Translation2d stationLocation;

    private STATE currentState;

    private final Timer autoTimer;
    private Timer lastTagTimestamp;

    private LimelightRunner limelightRunner;
    private final double kp = 0.9;


    private final Translation2d speed = new Translation2d(-1.0,0.0);
    // Robot State

    enum STATE {
        DRIVE,
        PREPARE,
        EXTEND,
        SHOOT,
        FINISHED
    }


    public DriveBackAndShootTest(TurretSubsystem turret, IntakeSubsystem intake, SwerveSubsystem swerve, SpindexerSubsystem spindexer){
        this.isCommandFinished = false;
        this.turret = turret;
        this.swerve = swerve;
        this.spindexer = spindexer;
        this.intake = intake;
        this.autoTimer = new Timer();

        this.lastTagTimestamp = new Timer();
        this.limelightRunner = LimelightRunner.getInstance();


        addRequirements(turret,intake,swerve,spindexer);
    }


    @Override
    public void initialize(){
        currentState = STATE.DRIVE;
        swerve.zeroGyroWithAlliance();
        autoTimer.start();
        lastTagTimestamp.start();


        boolean isRedAlliance = (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red);
        if (isRedAlliance) {
            this.stationLocation = Constants.StructureConstants.RED_SCORING_LOCATION;
        } else {
            this.stationLocation = Constants.StructureConstants.BLUE_SCORING_LOCATION;
        }
    }


    //FOR REFERENCE:
//            autoChooser.addOption(
//                    "Back Up and Shoot",
//                    Commands.runOnce(drivebase::zeroGyroWithAlliance).withTimeout(.2)
//                        .andThen( turretSubsystem.setShooterMotor(3000).withTimeout(1))
//            .andThen(drivebase.driveBackward().withTimeout(1.0))
//            .andThen(Commands.run(()->turretSubsystem.moveHoodUp(5,0.1)).withTimeout(0.6))
//            .andThen(Commands.run(spindexerSubsystem::feed, spindexerSubsystem).withTimeout(10.0))
//
//            );

    @Override
    public void execute(){
        double[] targetingInfo = limelightRunner.getTurretTagBasicInfo();


        double status = targetingInfo[0];
        double xOffset = targetingInfo[1];
        double distance = targetingInfo[2];

        switch ( currentState ) {
            case DRIVE:
                // Drive to location
                swerve.drive(speed,0.0,false);

                if(autoTimer.get() >= 1.0){
                    autoTimer.reset();
                    currentState = STATE.PREPARE;
                }

                break;
            case PREPARE:
                // Spin up motor, and aim
//                turret.updateShooterSpeed(LimelightRunner.getInstance().getTurretVelocityCameraToAprilTag());
//                turret.updateHoodAngle(LimelightRunner.getInstance().getHoodAngleCameraToAprilTag());

//                double currentX = SmartDashboard.getNumber("Swerve_X_Position",-67); //This should NEVER EVER not get a number!!!
//                double currentY = SmartDashboard.getNumber("Swerve_Y_Position",-67);
//
//                Translation2d robotPose = new Translation2d(currentX,currentY);

//                turret.updateShooterSpeed(LimelightRunner.getInstance().getTurretVelocityUsingPose(robotPose, stationLocation));
//                turret.updateHoodAngle(LimelightRunner.getInstance().getHoodAngleUsingPose(robotPose, stationLocation));

//                turret.updateHoodAngle(10);
//                turret.updateShooterSpeed(2800);


                if ( status >= 0.0) {
                    lastTagTimestamp.restart();
                    try {
                        int adjustedDist = (int) distance;
                        double currTurretPosition = turret.getTurretPosition();
                        double desiredAngle = currTurretPosition + (xOffset * kp);

                        turret.updateTurretRotation(desiredAngle);
                        turret.updateShooterSpeed(LimelightRunner.getInstance().getTurretVelocityCameraToAprilTag());
                        turret.updateHoodAngle(LimelightRunner.getInstance().getHoodAngleCameraToAprilTag());
                    } catch ( NullPointerException e ) {
                    }
                }

                if (lastTagTimestamp.hasElapsed(1.0)){
                    // reset to zero
                    turret.updateShooterSpeed(0.0);
                    turret.updateHoodAngle(0.0);
                    turret.updateTurretRotation(0.0);
                }


                if(autoTimer.get() >= 1.0) {
                    autoTimer.reset();
                    currentState = STATE.EXTEND;
                }
                break;
            case EXTEND:
                intake.extendIntake();
                if(autoTimer.get()>=0.3){
                    autoTimer.reset();
                    currentState = STATE.SHOOT;
                }
                break;
            case SHOOT:
                // Start spindexer, shoot, and shuffle
                spindexer.feed();
                if(autoTimer.get() >= 5.0){
                    currentState = STATE.FINISHED;
                }
                break;
            case FINISHED:
                // Cleanup, set to defaults, set inturrupt
                isCommandFinished = true;
                break;
        }

    }


    @Override
    public boolean isFinished(){
        return isCommandFinished;
    }


    @Override
    public void end(boolean interrupted) {
        swerve.lock();
        spindexer.stopMotors();
        turret.updateHoodAngle(0.0);
        turret.updateTurretRotation(0.0);
        turret.updateShooterSpeed(0.0);
    }
}


