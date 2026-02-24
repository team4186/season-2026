package frc.robot.vision;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LimelightRunner extends SubsystemBase {
    private final DoubleSubscriber tvSub;
    private final DoubleSubscriber txSub;
    private final DoubleSubscriber tySub;
    private final DoublePublisher ledPub;


    public LimelightRunner(){
        NetworkTable turretTable = NetworkTableInstance.getDefault().getTable("limelight-turret");
        NetworkTable robotTable = NetworkTableInstance.getDefault().getTable("limelight-robot");

        tvSub = turretTable.getDoubleTopic("tv").subscribe(0.0);
        txSub = turretTable.getDoubleTopic("tx").subscribe(0.0);
        tySub = turretTable.getDoubleTopic("ty").subscribe(0.0);
        ledPub = turretTable.getDoubleTopic("ledMode").publish();
    }

    @Override
    public void periodic() {
        boolean tagInView = hasTargetTag();
        SmartDashboard.putBoolean("Has Target Tag?", tagInView);
        ledPub.set( tagInView? 3.0 : 1.0 );

        SmartDashboard.putNumber("tx", txSub.get());
        SmartDashboard.putNumber("ty", tySub.get());
//        SmartDashboard.putNumber("X Offset", tagOffset)
//        SmartDashboard.putNumber("Y Offset", yOffset)
//        SmartDashboard.putNumber("% of Image", tagArea)
//        SmartDashboard.putNumber("Distance", Units.metersToInches(distance))
    }


    private boolean hasTargetTag(){ return tvSub.get() > 0.0; }


    public void close(){
        tvSub.close();
        txSub.close();
        tySub.close();
        ledPub.close();
    }
}
