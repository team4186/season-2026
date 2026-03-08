package frc.robot.subsystems;

import com.revrobotics.spark.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.vision.LimelightRunner;
import java.lang.Math.*;

public class TurretSubsystem extends SubsystemBase {
    // Shooter motor
    private final SparkFlex shooterMotor;

    // Turret motor
    private final SparkMax aimingMotor;

    // Shooter closed loop controller (slot 0: position, slot 1: velocity)
    private final SparkClosedLoopController shooterClosedLoopController;

    // aiming closed loop controller (slot 0: position, slot 1: velocity)
    private final SparkClosedLoopController aimingClosedLoopController;

    private final SparkClosedLoopController hoodClosedLoopController;

    // max rotation (170 degrees, with 20 degrees of dead zone in the back)
    private double maxRotation = Constants.TurretConstants.MAX_ROTATION;

    // min rotation (-170 degrees, with 20 degrees of dead zone in the back)
    private double minRotation = -Constants.TurretConstants.MAX_ROTATION;

    // zero limit switch (for turret)
    private final DigitalInput zeroLimitSwitch;
    // left limit switch
    private final DigitalInput leftLimitSwitch;
    // right limit switch
    private final DigitalInput rightLimitSwitch;

    private final LimelightRunner limelightRunner;

    // Leave unimplemented for now until it is designed
    // Loader motor
    // Spindexer motor
    // Hood motor
    private final SparkMax hoodMotor;
    // home limit switch (starting pos for hood)
    private final DigitalInput homeLimitSwitch;
    // hard-stop max extended position


    public TurretSubsystem(
            SparkFlex shooterMotor,
            SparkMax aimingMotor,
            SparkMax hoodMotor,
            DigitalInput zeroLimitSwitch,
            DigitalInput leftLimitSwitch,
            DigitalInput rightLimitSwitch,
            DigitalInput homeLimitSwitch,
            LimelightRunner limelightRunner
    ){
        this.shooterMotor = shooterMotor;
        this.aimingMotor = aimingMotor;
        this.hoodMotor = hoodMotor;

        this.shooterClosedLoopController = shooterMotor.getClosedLoopController();
        this.aimingClosedLoopController = aimingMotor.getClosedLoopController();
        this.hoodClosedLoopController = hoodMotor.getClosedLoopController();

        this.zeroLimitSwitch = zeroLimitSwitch;
        this.leftLimitSwitch = leftLimitSwitch;
        this.rightLimitSwitch = rightLimitSwitch;
        this.homeLimitSwitch = homeLimitSwitch;

        this.limelightRunner = limelightRunner;
    }


    @Override
    public void periodic(){}


    public void stopMotors(){
        shooterMotor.stopMotor();
        aimingMotor.stopMotor();
        hoodMotor.stopMotor();
    }


    public void reset() {
        shooterMotor.set(0); // TODO: update to set kVelocity using closed loop controller
        while (!homeLimitSwitch.get()) { // TODO: Rewrite while loop to check state and update incrementally
            hoodMotor.set(-0.5);
        }

        while (!zeroLimitSwitch.get()) { // TODO: Rewrite while loop to check state and update incrementally
            // Apply power to motor directly
            // aimingMotor.set();

            // Update desired position for motor controller
            aimingClosedLoopController.setSetpoint(0, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }
    }


    // TODO: What is the default case for each turret limit switch? Should we track and/or note this here?
    private boolean getLeftLimitSwitch() {
        return leftLimitSwitch.get();
    }

    private boolean getRightLimitSwitch() {
        return rightLimitSwitch.get();
    }

    private boolean getZeroLimitSwitch() { return zeroLimitSwitch.get(); }

    private boolean getHomeLimitSwitch() { return homeLimitSwitch.get(); }

    /*
     TODO: Should we update our setpoint difference from where we are to where we want to be? 0 -> 0+20 or 15 -> 15-35
     Where should we check for edge case if our desired location is past our hard stop?
     */


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


    // TODO: Decision -> Should we include this in reset function? Should we have separate reset functions with one calling all of them in one case?

    // TODO: Implement with closed loop controller and desired rpm or speed if using velocity conversion with encoder

    public void updateTurretRotation(double angle) {
        aimingClosedLoopController.setSetpoint(filter(angle), SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void updateShooterSpeed(double rpm) {
        shooterClosedLoopController.setSetpoint(rpm, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot1);
    }

    public void updateHoodAngle(double angle) {
        hoodClosedLoopController.setSetpoint(Math.max(0, Math.min(angle, 35)), SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    // switch this to switch case
    private double filter (double reqSetpoint) {
        double actualSetpoint = 0;
        if (reqSetpoint >= 170 && reqSetpoint < 190) {
            actualSetpoint = 170;
        } else if (reqSetpoint <= -170 && reqSetpoint > -190) {
            actualSetpoint = -170;
        } else if (reqSetpoint >= 190) {
            actualSetpoint = reqSetpoint - 360;
        } else if (reqSetpoint < -190) {
            actualSetpoint = reqSetpoint + 360;
        }
        return actualSetpoint;
    }
}
