package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
    // Shooter motor

    // Turret motor
    // max rotation
    // zero limit switch
    // left limit switch
    // right limit switch

    // Loader motor

    // Spindexer motor

    // Hood motor
    // home limit switch (starting pos)
    // hard-stop max extended position


    public TurretSubsystem(){}


    @Override
    public void periodic(){}


    public void stopMotors(){
        // stop turret motor
        // stop shooter motor
    }


    public void reset(){}


    private boolean getLeftBeamBreak(){ return false; }


    private boolean getRightBeamBreak(){ return false; }


    public void updateTurretSetpoint(double setpoint){}


    public void updateShooterRpm(double rpm){}
}
