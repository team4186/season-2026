package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
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


    public TurretSubsystem(SparkFlex shooterMotor, SparkMax aimingMotor, SparkMax hoodMotor, DigitalInput zeroLimitSwitch,
                           DigitalInput leftLimitSwitch, DigitalInput rightLimitSwitch,
                           DigitalInput homeLimitSwitch){
        this.shooterMotor = shooterMotor;
        this.aimingMotor = aimingMotor;
        this.shooterClosedLoopController = shooterMotor.getClosedLoopController();
        this.aimingClosedLoopController = shooterMotor.getClosedLoopController();
        this.zeroLimitSwitch = zeroLimitSwitch;
        this.leftLimitSwitch = leftLimitSwitch;
        this.rightLimitSwitch = rightLimitSwitch;
        this.homeLimitSwitch = homeLimitSwitch;
        this.hoodMotor = hoodMotor;
    }


    @Override
    public void periodic(){}


    public void stopMotors(){
        shooterMotor.set(0);
        aimingMotor.set(0);
        hoodMotor.set(0);
    }


    public void reset() {
        shooterMotor.set(0);
        while (!homeLimitSwitch.get()) {
            hoodMotor.set(-0.5);
        }
        while (!zeroLimitSwitch.get()) {
            aimingMotor.set(aimingClosedLoopController.setSetpoint(0, ControlType.kPosition));
        }
    }


    private boolean getLeftLimitSwitch() {
        return leftLimitSwitch.get();
    }

    private boolean getRightLimitSwitch() {
        return rightLimitSwitch.get();
    }


    public void updateTurretSetpoint(double setpoint) {
        //Grab angle offset from april tag, use that as the current value and
        // make PID set point 0.
    }

    public void getRegression(double distanceOffset) {
        /**Create a regression function during testing of optimal hood angle
        vs RPM at different distances. 
        **/
    }

    public void updateHoodSetpoint(double setpoint) {
        // setpoint
    }

    public void updateShooterRpm(double setpoint){
        // setpoint should ideally be from the regression table
    }
}
