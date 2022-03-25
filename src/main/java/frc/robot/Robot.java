// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.TestCommand;
import frc.robot.SwerveDrive.SwerveDrive;


/**
 * The VM is configured to automatically run this class. If you change the name of this class or the
 * package after creating this project, you must also update the build.gradle file in the project.
 */
public class Robot extends RobotBase
{

    private final SwerveDrive m_swerve = new SwerveDrive();
    private final Characterizer m_characterizer = new Characterizer(this);

    public void robotInit() {
        m_swerve.setDefaultCommand(new TestCommand(m_swerve));
        ShuffleboardManager.init(this);

    }
    
    
    public void disabled() {}
    
    
    public void autonomous() {}
    
    
    public void teleop() {
    }

    public void test() {
    }

    public void testPeriodic() {
    }
    
    
    private volatile boolean exit;
    
    
    @Override
    public void startCompetition()
    {
        robotInit();

        // Tell the DS that the robot is ready to be enabled
        HAL.observeUserProgramStarting();
        
        while (!Thread.currentThread().isInterrupted() && !exit)
        {
            Shuffleboard.update();

            if (isDisabled())
            {
                DriverStation.inDisabled(true);
                disabled();
                DriverStation.inDisabled(false);
                while (isDisabled())
                {
                    DriverStation.waitForData();
                }
            }
            else if (isAutonomous())
            {
                DriverStation.inAutonomous(true);
                autonomous();
                DriverStation.inAutonomous(false);
                while (isAutonomousEnabled())
                {
                    periodic();
                    DriverStation.waitForData();
                }
            }
            else if (isTest())
            {
                LiveWindow.setEnabled(true);
                Shuffleboard.enableActuatorWidgets();
                DriverStation.inTest(true);
                test();
                DriverStation.inTest(false);
                while (isTest() && isEnabled())
                {
                    testPeriodic();
                    periodic();
                    DriverStation.waitForData();
                }
                LiveWindow.setEnabled(false);
                Shuffleboard.disableActuatorWidgets();
            }
            else
            {
                DriverStation.inTeleop(true);
                teleop();
                DriverStation.inTeleop(false);
                while (isTeleopEnabled())
                {
                    Shuffleboard.update();
                    periodic();
                    DriverStation.waitForData();
                }
            }
        }
    }
    
    
    @Override
    public void endCompetition()
    {
        exit = true;
    }

    private void periodic() {
        m_characterizer.periodic();
        CommandScheduler.getInstance().run();
        Shuffleboard.update();
    }

    public SwerveDrive getSwerve() {
        return m_swerve;
    }

    public Characterizer getCharacterizer() {
        return m_characterizer;
    }
}
