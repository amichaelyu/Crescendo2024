package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.controller.BetterXboxController;
import frc.lib.controller.BetterXboxController.Humans;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TilterConstants;
import frc.robot.commands.*;
import frc.robot.commands.old.TeleIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Tilter;

public class RobotContainer {
    /* Controllers */
    public static BetterXboxController driver = new BetterXboxController(0, Humans.DRIVER);
    public static BetterXboxController operator = new BetterXboxController(1, Humans.OPERATOR);

    /* Driver Buttons */
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        NamedCommands.registerCommand("speakerShot", new CG_ShootingSpeaker());
        NamedCommands.registerCommand("intake", new IntakeRun());
        NamedCommands.registerCommand("citrusShot", new CG_ShootingLime());
        NamedCommands.registerCommand("citrusPose", new SwerveLimePose());

        autoConfig();

//        configureButtonBindings();
        competitionButtons();

        Swerve.getInstance().setDefaultCommand(new TeleopSwerve(driver.leftBumper()));
        Indexer.getInstance().setDefaultCommand(new IndexerManual(() -> (0.25 * (driver.getRightTriggerAxis()-driver.getLeftTriggerAxis()))));
        Tilter.getInstance().setDefaultCommand(new TilterManual(() -> ((operator.getRightTriggerAxis()- operator.getLeftTriggerAxis()))));
        Climber.getInstance().setDefaultCommand(new ClimberManual(() -> (operator.getRawAxis(5))));
    }

    private void competitionButtons() {
        driver.a().whileTrue(new CG_ShootingLime())
                .whileFalse(new TilterSetpointPosition(TilterConstants.GROUND_INTAKE_POSITION));
//        driver.b()
        driver.x().whileTrue(new CG_FlushShots());
        driver.y().onTrue(new InstantCommand(Swerve.getInstance()::zeroHeading));

        driver.rightBumper().whileTrue(new CG_IntakeIndexer())
                            .whileFalse(new IndexerSlightBack());

//        operator.rightBumper().whileTrue(new CG_ShootingIntake());

        operator.leftBumper().whileTrue(new TilterHome());
        operator.rightBumper().whileTrue(new CG_ShootingIntake())
                .whileFalse(new TilterSetpointPosition(TilterConstants.GROUND_INTAKE_POSITION));

        // uses these for obtaining new data
        operator.a().whileTrue(new TilterDashboardPosition());
        operator.b().whileTrue(new ShooterDashboardSpeed());
        operator.x().whileTrue(new IndexerKick());


        // these are normal operator buttons
//        operator.y().whileTrue(new CG_ShootingSpeaker())
//                .whileFalse(new TilterSetpointPosition(TilterConstants.GROUND_INTAKE_POSITION));
//        operator.b().whileTrue(new CG_ShootingAmp())
//                .whileFalse(new TilterSetpointPosition(TilterConstants.GROUND_INTAKE_POSITION));
    }

    private void configureButtonBindings() {

//        driver.a().whileTrue(new TilterDashboardPosition());
        driver.a().whileTrue(new CG_ShootingLime());
        driver.b().whileTrue(new SwerveRotateLime());
//        driver.a().whileTrue(new ShooterDashboardSpeed());
//        driver.b().whileTrue(new TilterDashboardPosition());
//        driver.x().whileTrue(new IndexerRun());

        /* Driver Buttons */
        driver.y().onTrue(new InstantCommand(Swerve.getInstance()::zeroHeading));

//        operator.leftBumper().whileTrue(new TeleShooter());

        //run shooter wheels as intake
        operator.rightBumper().whileTrue(new ShooterSetpointSpeed(ShooterConstants.INTAKE_SPEED));

        //run intake forward
        driver.rightBumper().whileTrue(new TeleIntake());
    }

    private void autoConfig() {
        autoChooser.addOption("Nothing", new WaitCommand(0));
        autoChooser.addOption("Speaker Shot", new CG_ShootingSpeaker());
        autoChooser.addOption("Top Speaker Shot and Escape", AutoBuilder.buildAuto("top 1 + 1"));
        autoChooser.addOption("3 note", AutoBuilder.buildAuto("3 note"));

        SmartDashboard.putData("Auto Command", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}