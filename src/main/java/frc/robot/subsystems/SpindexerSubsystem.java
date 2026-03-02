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

    public rotateSpindexer(){
        rotateMotor.accept(SpindexerConstants.MAX_SPEED);
    }

    @Override
    public void periodic(){}

    public void spin(){}

    public void feed(){}

}
