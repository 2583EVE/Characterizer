package frc.robot.Commands;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SwerveDrive.SwerveDrive;

public class TestCommand extends CommandBase {

    private final SwerveDrive swerveDrive;
    public TestCommand(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        swerveDrive.drive(0, 0, 1, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
