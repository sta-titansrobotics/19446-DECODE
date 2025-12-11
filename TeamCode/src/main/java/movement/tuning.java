package movement;

import com.acmerobotics.dashboard.config.Config;


@Config
public class tuning {
    public static double rotkp = 0.0075;
    public static double rotkd = 0.01;
    public static double rotkp2 = 0.03;
    public static double rotkd2 = 0.01;
    public static double rotkp3 = 0.05;
    public static double rotkd3 = 0.01;
    public static double rotmin = 0.;
    public static double rotmin2 = 0.085;
    public static double rotmin3 = 0.05;
    public static double rotthresh = 15;
    public static double rotthresh2 = 5;
    public static double aprilcorr = 0.1;
    //public double rotkp = 0;
}
