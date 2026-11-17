package pedroPathing;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ConfigValues {

    public static double kP = 0.00003;
    public static double kD = 0.00004;
    public static double LOOP_TIME = 0.025;

    public static double high = 1;
    public static double closeZone = 350;

    public static double transferHigh = 1.0;
    public static double transferLow = 0.6;

    public static double gateClosed = 0.67;
    public static double gateOpen = 0.5;

    public static double tongueHigh = 0.34;
    public static double tongueLow = 0.95;

    public static double pN = 0.0034;
    public static double dN = 0;
}