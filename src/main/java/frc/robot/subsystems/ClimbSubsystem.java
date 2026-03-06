package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ClimbSubsystem extends SubsystemBase {
    // left climb motor
    // right climb motor
    // pigeon/gyro --- swerve for gyro instance periodically
    // retracted limit switch
    private int endOfTravel;


    public ClimbSubsystem(
            int endOfTravelConst
    ) {
        this.endOfTravel = endOfTravelConst;
    }


    @Override
    public void periodic(){}


    public void climbUp(){}


    public void climbDown(){}


    public void climbHold(){}


    public void climbStop(){}


    public void climbReset(){}
}
