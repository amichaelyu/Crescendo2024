package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.controller.BetterXboxController;
import frc.lib.controller.BetterXboxController.Humans;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    public static BetterXboxController driver = new BetterXboxController(0, Humans.DRIVER);
    public static BetterXboxController operator = new BetterXboxController(1, Humans.OPERATOR);

    /* Driver Buttons */
    private final Trigger zeroGyro = driver.y();
    private final Trigger robotCentric = driver.leftBumper();

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Shooter m_shooter = new Shooter();
    private final Indexer m_indexer = new Indexer();
    private final Intake m_intake = new Intake();
    private final Tilter m_tilter = new Tilter();
    private final Climber m_climber = new Climber();
    private final Limelight m_limelight = new Limelight();



    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                robotCentric
            )
        );

        configureButtonBindings();

        m_indexer.setDefaultCommand(new TeleIndexer(m_indexer, () -> ((driver.getRightTriggerAxis()-driver.getLeftTriggerAxis()))));
        m_tilter.setDefaultCommand(new TeleTilter(m_tilter, () -> ((operator.getRightTriggerAxis()- operator.getLeftTriggerAxis()))));
        m_climber.setDefaultCommand(new TeleClimber(m_climber, () -> (operator.getRawAxis(5))));

    }


    private void configureButtonBindings() {
        /* Driver Buttons */
        driver.y()
            .onTrue(new InstantCommand(s_Swerve::zeroHeading));

        operator.leftBumper().whileTrue(new TeleShooter(m_shooter));

        //run shooter wheels as intake
        operator.rightBumper().whileTrue(new RepeatCommand(new ShooterIntake(m_shooter)));

        //run intake forward
        driver.rightBumper().whileTrue(new TeleIntake(m_intake));
        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}