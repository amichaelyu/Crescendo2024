package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    public static final CommandXboxController driver = new CommandXboxController(0);
    public static final CommandXboxController codriver = new CommandXboxController(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

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



    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();

        codriver.leftBumper().whileTrue(new TeleShooter (m_shooter));
        //run shooter wheels as intake
        codriver.rightBumper().whileTrue(m_shooter.getIntakeCommand());
        //run intake forward
        driver.leftBumper().whileTrue(new TeleIntake (m_intake));


        m_indexer.setDefaultCommand(new TeleIndexer(m_indexer, () -> ((driver.getRightTriggerAxis()-driver.getLeftTriggerAxis()))));
        m_tilter.setDefaultCommand(new TeleTilter(m_tilter, () -> ((codriver.getRightTriggerAxis()-codriver.getLeftTriggerAxis()))));
        m_climber.setDefaultCommand(new TeleClimber(m_climber, () -> (codriver.getRightY())));

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
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
