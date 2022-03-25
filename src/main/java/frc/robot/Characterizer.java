package frc.robot;

public class Characterizer {

    private Robot robot;

    private double p = 10,
                   i = 0,
                   d = 0;

    private double a = 0,
                   s = 0.5,
                   v = 0;

    public Characterizer(Robot robot) {
        this.robot = robot;
    }

    public void periodic() {
        robot.getSwerve().updatePID(p, i, d);
        robot.getSwerve().updateFeedForward(a, s, v);

        ShuffleboardManager.updateDouble("p", p);
        ShuffleboardManager.updateDouble("i", i);
        ShuffleboardManager.updateDouble("d", d);

        ShuffleboardManager.updateDouble("a", a);
        ShuffleboardManager.updateDouble("s", s);
        ShuffleboardManager.updateDouble("v", v);
    }

    // SETTERS

    public void setP(double p) {
        this.p = p;
    }

    public void setI(double i) {
        this.i = i;
    }

    public void setD(double d) {
        this.d = d;
    }

    public void setA(double a) {
        this.a = a;
    }

    public void setS(double s) {
        this.s = s;
    }

    public void setV(double v) {
        this.v = v;
    }

    // GETTERS

    public double getP() {
        return p;
    }

    public double getI() {
        return i;
    }

    public double getD() {
        return d;
    }

    public double getA() {
        return a;
    }

    public double getS() {
        return s;
    }

    public double getV() {
        return v;
    }
}
