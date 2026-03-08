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


    // TODO: Should we rename function? Also do we want to set power manually or leverage closed loop controller
    // (Hint) How much do we care about maintaining consistent feeding and rotation speed?
    public void rotateSpindexer(){
        rotateMotor.set(SpindexerConstants.ROTATE_MAX_SPEED);
        feedMotor.set(SpindexerConstants.FEED_MAX_SPEED); //can create new constant that's different if needed
    }

    public void feedSpindexer(){

    }

    // TODO: What parts of the subsystem should we track and publish here?
    @Override
    public void periodic(){}

    public void spin(){}

    public void feed(){}

}
