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

    // zero limit switch
    private final DigitalInput zeroLimitSwitch;
    // left limit switch
    private final DigitalInput leftLimitSwitch;
    // right limit switch
    private final DigitalInput rightLimitSwitch;


    // Leave unimplemented for now until it is designed
    // Loader motor
    // Spindexer motor
    // Hood motor
    // home limit switch (starting pos)
    // hard-stop max extended position


    public TurretSubsystem(SparkFlex shooterMotor, SparkMax aimingMotor, DigitalInput zeroLimitSwitch, DigitalInput leftLimitSwitch, DigitalInput rightLimitSwitch){
        this.shooterMotor = shooterMotor;
        this.aimingMotor = aimingMotor;
        this.shooterClosedLoopController = shooterMotor.getClosedLoopController();
        this.aimingClosedLoopController = shooterMotor.getClosedLoopController();
        this.zeroLimitSwitch = zeroLimitSwitch;
        this.leftLimitSwitch = leftLimitSwitch;
        this.rightLimitSwitch = rightLimitSwitch;
    }


    @Override
    public void periodic(){}


    public void stopMotors(){
        shooterMotor.set(0);
        aimingMotor.set(0);
    }


    public void reset(){}


    private boolean getLeftBeamBreak(){ return false; }


    private boolean getRightBeamBreak(){ return false; }


    public void updateTurretSetpoint(double setpoint){}


    public void updateShooterRpm(double rpm){}
}
