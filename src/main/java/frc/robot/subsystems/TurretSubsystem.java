package frc.robot.subsystems;

import com.revrobotics.spark.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
    // Shooter motor
    private final SparkFlex shooterMotor;

    // Turret motor
    private final SparkMax aimingMotor;

    // Shooter closed loop controller (slot 0: position, slot 1: velocity)
    private final SparkClosedLoopController shooterClosedLoopController;

    // aiming closed loop controller (slot 0: position, slot 1: velocity)
    private final SparkClosedLoopController aimingClosedLoopController;

    // max rotation (170 degrees, with 20 degrees of dead zone in the back)
    private double maxRotation = Constants.TurretConstants.TURRET_MAX_ROTATION;

    // min rotation (-170 degrees, with 20 degrees of dead zone in the back)
    private double minRotation = -Constants.TurretConstants.TURRET_MAX_ROTATION;

    // zero limit switch (for turret)
    private final DigitalInput zeroLimitSwitch;
    // left limit switch
    private final DigitalInput leftLimitSwitch;
    // right limit switch
    private final DigitalInput rightLimitSwitch;


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
            DigitalInput homeLimitSwitch
    ){
        this.shooterMotor = shooterMotor;
        this.aimingMotor = aimingMotor;
        this.hoodMotor = hoodMotor;

        this.shooterClosedLoopController = shooterMotor.getClosedLoopController();
        this.aimingClosedLoopController = shooterMotor.getClosedLoopController();

        this.zeroLimitSwitch = zeroLimitSwitch;
        this.leftLimitSwitch = leftLimitSwitch;
        this.rightLimitSwitch = rightLimitSwitch;
        this.homeLimitSwitch = homeLimitSwitch;
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


    /*
     TODO: Should we update our setpoint difference from where we are to where we want to be? 0 -> 0+20 or 15 -> 15-35
     Where should we check for edge case if our desired location is past our hard stop?
     */
    public void updateTurretSetpoint(double setpoint) {
        //Grab angle offset from april tag, use that as the current value and
        // make PID set point 0.
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
    
    }


    // TODO: Decision -> Should we include this in reset function? Should we have separate reset functions with one calling all of them in one case?
    public void updateHoodSetpoint(double setpoint) {
        // setpoint should ideally be from regression table.
    }

    // TODO: Implement with closed loop controller and desired rpm or speed if using velocity conversion with encoder
    public void updateShooterRPM(double setpoint){
        // setpoint should ideally be from the regression table.
    }
}
