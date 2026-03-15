package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import java.lang.Math.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class TurretSubsystem extends SubsystemBase {
    // Motors
    private final SparkFlex shooterMotor;
    private final SparkMax turretMotor;
    private final SparkMax hoodMotor;

    // Shooter closed loop controller (slot 0: position)
    private final SparkClosedLoopController turretClosedLoopController;
    private final SparkClosedLoopController hoodClosedLoopController;

    // aiming closed loop controller (slot 1: Velocity)
    private final SparkClosedLoopController shooterClosedLoopController;

    // Turret and Hood Rotation
    private final double maxTurretRotation = TurretConstants.TURRET_MAX_ROTATION;
    private final double minTurretRotation = TurretConstants.TURRET_MIN_ROTATION;
    private final double maxHoodRotation = TurretConstants.HOOD_MAX_ROTATION;
    private final double minHoodRotation = TurretConstants.HOOD_MIN_ROTATION;

    private final double turretDeadZone = TurretConstants.TURRET_ROTATION_DEAD_ZONE;

    // Limit Switches
    private final DigitalInput hoodLimitSwitch;
    private final DigitalInput leftLimitSwitch;
    private final DigitalInput rightLimitSwitch;

    // Encoders
    private final RelativeEncoder turretRelativeEncoder;
    private final RelativeEncoder hoodRelativeEncoder;
    private final RelativeEncoder shooterRelativeEncoder;


    public TurretSubsystem(
            SparkFlex shooterMotor,
            SparkMax turretMotor,
            SparkMax hoodMotor,
            DigitalInput hoodLimitSwitch,
            DigitalInput leftLimitSwitch,
            DigitalInput rightLimitSwitch
    ){
        this.shooterMotor = shooterMotor;
        this.turretMotor = turretMotor;
        this.hoodMotor = hoodMotor;

        this.shooterClosedLoopController = shooterMotor.getClosedLoopController();
        this.turretClosedLoopController = turretMotor.getClosedLoopController();
        this.hoodClosedLoopController = hoodMotor.getClosedLoopController();

        this.hoodLimitSwitch = hoodLimitSwitch;
        this.leftLimitSwitch = leftLimitSwitch;
        this.rightLimitSwitch = rightLimitSwitch;

        this.turretRelativeEncoder = turretMotor.getEncoder();
        this.shooterRelativeEncoder = shooterMotor.getEncoder();
        this.hoodRelativeEncoder = hoodMotor.getEncoder();
    }


    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Turret_Left_Limit_Switch: ", getLeftLimitSwitch());
        SmartDashboard.putBoolean("Turret_Right_Limit_Switch: ", getRightLimitSwitch());
        SmartDashboard.putBoolean("Hood_Limit_Switch: ", getHoodLimitSwitch());

        SmartDashboard.putNumber("Shooter_Velocity: ", getShooterVelocity());
        SmartDashboard.putNumber("Turret_Position: ", getTurretPosition());
        SmartDashboard.putNumber("Hood_Position", getHoodPosition());

        SmartDashboard.putBoolean("Shooter_is_at_set_speed", isShooterAtSetpoint());
        SmartDashboard.putBoolean("Hood_is_at_set_angle", isHoodAtSetpoint());
        SmartDashboard.putBoolean("Turret_is_at_target_position", isTurretAtSetpoint());
    }


    public void stopMotors(){
        shooterMotor.stopMotor();
        turretMotor.stopMotor();
        hoodMotor.stopMotor();
    }


    public void updateTurretRotation(double angle) {
        turretClosedLoopController.setSetpoint(aimingFilter(angle), SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }


    public void updateShooterSpeed(double rpm) {
        shooterClosedLoopController.setSetpoint(rpm, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot1);
    }


    public void updateHoodAngle(double angle) {
        hoodClosedLoopController.setSetpoint(Math.max(minHoodRotation, Math.min(angle, maxHoodRotation)), SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }


    // switch this to switch case
    private double aimingFilter (double reqSetpoint) {
        double adjustedSetpoint = 0.0;

        if (reqSetpoint > maxTurretRotation + turretDeadZone ) {
            adjustedSetpoint = Math.max( reqSetpoint - 360, minTurretRotation);
        } else if (reqSetpoint < minTurretRotation - turretDeadZone) {
            adjustedSetpoint = Math.min( reqSetpoint + 360, maxTurretRotation);
        } else if (reqSetpoint >= maxTurretRotation) {
            adjustedSetpoint = maxTurretRotation;
        } else if (reqSetpoint <= minTurretRotation) {
            adjustedSetpoint = minTurretRotation;
        }

        return adjustedSetpoint;
    }


    // TODO: Simple table first then regression with enough data with high confidence
    public void getRegression(double detectedDistance) {
        /**Create a regression function during testing of optimal hood angle
         vs RPM at different distances.

         Feed limelight distance input into the regression function.
         **/

        /** Why is this better than a kinematics equation?
         To solve obtain the ideal shooting velocity for the updateShooterRPM function,
         you need know the angle and the launch distance from the turret to the wall.

         Getting the distance from the turret to the goal is simple enough, but getting the launch angle is tricky.
         The launch angle is not the same as the hood angle, so there is no easy way for us to know that on the fly.

         The kinematics equation outputs initial velocity in terms of m/s, but the robot takes velocity in RPM.
         While you can convert m/s to RPM, the ideal initial launch velocity for the ball is not the same as
         the initial velocity of the flywheel (which is roughly 2 times the speed).

         Not only that, the velocity of the flywheel isn't the same as the rotational velocity of the shaft.
         Meaning that you need to take that into account when calculating motor RPM.
         Even with all of that in consideration, there are still gear boxes, which makes this process even more
         complicated.

         After all that, you still need another kinematics equation to solve for the ideal launch angle.
         Which is just a pain to do for something that can be solved easily with regression.

         TDLR: Regression is simpler because we can ignore unit conversions from M/S to RPM and all
         the shenanigans that come with physics.
         **/

        /** Why is this worse than kinematics? Because I said so **/
    }

    public boolean getLeftLimitSwitch() { return leftLimitSwitch.get(); }


    public boolean getRightLimitSwitch() { return rightLimitSwitch.get(); }


    public boolean getHoodLimitSwitch() { return hoodLimitSwitch.get(); }


    public double getTurretPosition() { return turretRelativeEncoder.getPosition(); }


    public double getShooterVelocity() { return shooterRelativeEncoder.getVelocity(); }


    public double getHoodPosition() { return hoodRelativeEncoder.getPosition(); }


    public boolean isTurretAtSetpoint() { return turretClosedLoopController.isAtSetpoint(); }


    public boolean isShooterAtSetpoint() { return shooterClosedLoopController.isAtSetpoint(); }


    public boolean isHoodAtSetpoint() { return hoodClosedLoopController.isAtSetpoint(); }
}
