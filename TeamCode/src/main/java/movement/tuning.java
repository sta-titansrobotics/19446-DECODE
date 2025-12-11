package movement;

import com.acmerobotics.dashboard.config.Config;


@Config
public class tuning {
    public static double rotkp = 0.0045;
    public static double rotkd = 0.001;
    public static double rotkp2 = 0.01;
    public static double rotkd2 = 0.005;
    public static double rotmin = 0.;
    public static double rotmin2 = 0.04;
    public static double rotthresh = 20;
    public static double aprilcorr = 0;
    //public double rotkp = 0;
}
