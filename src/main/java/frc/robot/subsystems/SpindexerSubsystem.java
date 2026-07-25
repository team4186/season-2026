package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpindexerConstants;


public class SpindexerSubsystem extends SubsystemBase {
    private final SparkMax rotateMotor;
    private final SparkMax feedMotor;
    private final SparkMax assistMotor;
    private final RelativeEncoder feedEncoder;
    private final RelativeEncoder rotateEncoder;
    private final RelativeEncoder assistEncoder;

    private final SparkClosedLoopController assistCLController;

    public SpindexerSubsystem(SparkMax rotateMotor, SparkMax feedMotor, SparkMax assistMotor) {
        this.rotateMotor = rotateMotor;
        this.feedMotor = feedMotor;
        this.assistMotor = assistMotor;

        this.feedEncoder = feedMotor.getEncoder();
        this.rotateEncoder = rotateMotor.getEncoder();
        this.assistEncoder = assistMotor.getEncoder();

        this.assistCLController = assistMotor.getClosedLoopController();
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Spin_Feed_Velocity", feedEncoder.getVelocity());
        SmartDashboard.putNumber("Spin_Rotate_Velocity", rotateEncoder.getVelocity());
    }


    public void setAssistMotorSpeed(double speed) {
        assistCLController.setSetpoint(speed, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    }


    public void feed() {
        double shooterSpeed = SmartDashboard.getNumber("Shooter_Velocity:", 0.0);

        feedMotor.set(SpindexerConstants.FEED_MAX_SPEED);
        rotateMotor.set(SpindexerConstants.ROTATE_MAX_SPEED);
//        if (shooterSpeed >= 1000) {
//            feedMotor.set(SpindexerConstants.FEED_MAX_SPEED);
//            rotateMotor.set(SpindexerConstants.ROTATE_MAX_SPEED);
//        } else {
//            feedMotor.stopMotor();
//            rotateMotor.stopMotor();
//        }

        setAssistMotorSpeed(300.0);
    }

    public void stopMotors(){
        feedMotor.stopMotor();
        rotateMotor.stopMotor();
        assistCLController.setSetpoint(0.0, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    }


    public Command rotateMotors(){
        return Commands.runOnce( this::feed, this).repeatedly();
    }


    public Command stopFeed(){
        return Commands.runOnce( this::stopMotors , this);
    }
}
