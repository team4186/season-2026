package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SpindexerConstants;


public class SpindexerSubsystem extends SubsystemBase {
    private final SparkMax rotateMotor;
    private final SparkMax feedMotor;

    public SpindexerSubsystem(SparkMax rotateMotor, SparkMax feedMotor){
        this.rotateMotor = rotateMotor;
        this.feedMotor = feedMotor;

    }

    public void rotateSpindexer(){
        rotateMotor.set(SpindexerConstants.MAX_SPEED);
        feedMotor.set(SpindexerConstants.MAX_SPEED); //can create new constant that's different if needed
    }

    @Override
    public void periodic(){}

    public void spin(){}

    public void feed(){}

}
