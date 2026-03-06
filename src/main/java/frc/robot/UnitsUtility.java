package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

public class UnitsUtility {
    public static boolean isBeamBroken(DigitalInput limitSwitch, boolean defaultValue, String systemName ){
        try {
            return limitSwitch.get();
        } catch (IllegalStateException e) {
            String msg = systemName + " Beam break error: " + e;
            System.out.println( msg );
            return defaultValue;
        }
    }
}
