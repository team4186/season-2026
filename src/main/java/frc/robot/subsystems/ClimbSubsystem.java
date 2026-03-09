package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants.ClimbConstants;


public class ClimbSubsystem extends SubsystemBase {
    private final SparkMax climbMotor;
    private final DigitalInput homeSwitch;
    private final RelativeEncoder climbEncoder;
    private final SparkClosedLoopController climbMotorController;
    // left climb motor
    // right climb motor
    // pigeon/gyro --- swerve for gyro instance periodically
    // retracted limit switch
    private int endOfTravel;


    public ClimbSubsystem(
            SparkMax climbMotor,
            DigitalInput homeSwitch
    ) {
    this.climbMotor = climbMotor;
    this.homeSwitch = homeSwitch;
    this.climbEncoder = climbMotor.getEncoder();
    this.climbMotorController = climbMotor.getClosedLoopController();
    }


    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Climb Limit Switch: ", getLimitSwitch());
        SmartDashboard.putNumber("Climb Encoder Readings: ", readEncoder());
        SmartDashboard.putBoolean("Is climb at set point: ", climbMotorController.isAtSetpoint());
    }


    // Generic updateClimb function, setpoint can be some angle where the arm needs to be deployed.
    // Setpoint could also be some angle where the arm is clamped down.
    public void updateClimb(double angleSetpoint){
        climbMotorController.setSetpoint(angleSetpoint, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void climbStop(){
        climbMotor.stopMotor();
    }

    public double readEncoder(){
        return climbEncoder.getPosition();
    }

    public void resetClimb() {
        if (!getLimitSwitch()) {
            climbMotorController.setSetpoint(0, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
        } else {

            climbStop();
            resetEncoder();
        }
    }

    public void resetEncoder() {
        climbEncoder.setPosition(0);
    }

    public boolean getLimitSwitch(){
        return homeSwitch.get();
    }
}
