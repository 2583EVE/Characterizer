package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.*;
import frc.robot.Commands.TestCommand;

import java.util.Locale;
import java.util.stream.Collectors;

public class ShuffleboardManager {

    static ShuffleboardTab tab;
    static Robot robot;
    static ShuffleboardLayout PID,
                              feedForward;

    static NetworkTableEntry P,
                             I,
                             D,
                             A,
                             S,
                             V;

    static NetworkTableEntry error;

    public static void init(Robot robot) {
        tab = Shuffleboard.getTab("Drive Characterization");
        Shuffleboard.selectTab("Drive Characterization");
        ShuffleboardManager.robot = robot;

        addObject(robot.getSwerve());
        error = addObject("Error", 0.0).withWidget(BuiltInWidgets.kGraph).getEntry();

        tab.add(new TestCommand(robot.getSwerve()));

        PID = tab
                .getLayout("PID", BuiltInLayouts.kList)
                .withSize(4, 8)
                .withPosition(20, 0);

        P = addObject("P", robot.getCharacterizer().getP(), PID)
                .getEntry();

        I = addObject("I", robot.getCharacterizer().getI(), PID)
                .getEntry();

        D = addObject("D", robot.getCharacterizer().getD(), PID)
                .getEntry();

        feedForward = tab
                .getLayout("Feed Forward", BuiltInLayouts.kList)
                .withPosition(24, 0);;

        A = addObject("A", robot.getCharacterizer().getA(), feedForward)
                .getEntry();

        S = addObject("S", robot.getCharacterizer().getS(), feedForward)
                .getEntry();

        V = addObject("V", robot.getCharacterizer().getV(), feedForward)
                .getEntry();
    }

    private static ComplexWidget addObject(Sendable o) {
        String title = o.getClass().getSimpleName();
        System.out.println(title);
        if (!tab.getComponents().stream().map((c) -> c.getTitle()).collect(Collectors.toList()).contains(title))
            return tab.add(o);
        return (ComplexWidget) tab.getComponents().stream().filter((c) -> c.getTitle().equals(title)).collect(Collectors.toList()).get(0);
    }

    private static SimpleWidget addObject(String title, Object o) {
        if (!tab.getComponents().stream().map((c) -> c.getTitle()).collect(Collectors.toList()).contains(title))
            return tab.addPersistent(title, o);
        return (SimpleWidget) tab.getComponents().stream().filter((c) -> c.getTitle().equals(title)).collect(Collectors.toList()).get(0);
    }

    private static SimpleWidget addObject(String title, Object o, ShuffleboardLayout layout) {
        if (!layout.getComponents().stream().map((c) -> c.getTitle()).collect(Collectors.toList()).contains(title))
            return layout.addPersistent(title, o);
        return (SimpleWidget) layout.getComponents().stream().filter((c) -> c.getTitle().equals(title)).collect(Collectors.toList()).get(0);
    }

    public static void updateDouble(String entry, double value) {
        switch (entry.toUpperCase(Locale.ROOT)) {
            case "ERROR":
                error.setDouble(value);
                break;
            case "P":
                P.setDouble(value);
                break;
            case "I":
                I.setDouble(value);
                break;
            case "D":
                D.setDouble(value);
                break;
            case "A":
                A.setDouble(value);
                break;
            case "S":
                S.setDouble(value);
                break;
            case "V":
                V.setDouble(value);
                break;
        }
    }
}
