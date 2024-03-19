package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.controller.BetterXboxController;
import frc.lib.controller.BetterXboxController.Humans;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TilterConstants;
import frc.robot.commands.*;
import frc.robot.commands.old.TeleIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Flipper;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Tilter;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;

public class RobotContainer {
    /* Controllers */
    public static BetterXboxController driver = new BetterXboxController(0, Humans.DRIVER);
    public static BetterXboxController operator = new BetterXboxController(1, Humans.OPERATOR);

    /* Driver Buttons */
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        NamedCommands.registerCommand("speakerShot", new CG_ShootingSpeaker());
//        NamedCommands.registerCommand("tilterSetpointLow", new TilterSetpointPosition(5));
        NamedCommands.registerCommand("shooterPrep", new ShooterAutoPrep());
        NamedCommands.registerCommand("tilterPoseSetpoint", new TilterPose());
        NamedCommands.registerCommand("tilterIdle", new TilterSetpointPosition(TilterConstants.IDLE_POSITION));
        NamedCommands.registerCommand("intake", new CG_IntakeIndexer());
        NamedCommands.registerCommand("slightBack", new IndexerSlightBack());
        NamedCommands.registerCommand("citrusShot", new CG_ShootingLime());
        NamedCommands.registerCommand("citrusShotNoRot", new CG_ShootingLimeNoRot());

        autoConfig();

//        configureButtonBindings();
        competitionButtons();

//        operator.rightBumper().whileTrue(new InstantCommand(Swerve.getInstance()::driveForward));
//
//        operator.x().whileTrue(tilterRoutine.dynamic(Direction.kForward));
//        operator.y().whileTrue(tilterRoutine.dynamic(Direction.kReverse));
//        operator.a().whileTrue(tilterRoutine.quasistatic(Direction.kForward));
//        operator.b().whileTrue(tilterRoutine.quasistatic(Direction.kReverse));

        Swerve.getInstance().setDefaultCommand(new SwerveXboxDrive(driver.leftBumper()));
        Tilter.getInstance().setDefaultCommand(new TilterManual(() -> ((operator.getRightTriggerAxis() - operator.getLeftTriggerAxis()))));
        Climber.getInstance().setDefaultCommand(new ClimberManual(() -> (operator.getRightY())));
        Flipper.getInstance().setDefaultCommand(new RepeatCommand(new InstantCommand(() -> Flipper.getInstance().dutyCycle(operator.getLeftY()), Flipper.getInstance())));
    }

    private void competitionButtons() {
        driver.a().whileTrue(new CG_ShootingLime())
                .whileFalse(new TilterSetpointPosition(TilterConstants.IDLE_POSITION));
        driver.x().whileTrue(new CG_FlushShots());
        driver.y().onTrue(new InstantCommand(Swerve.getInstance()::zeroHeading));

        driver.rightBumper().whileTrue(new CG_IntakeIndexer())
                            .whileFalse(
                                    new ParallelCommandGroup(
                                        new IndexerSlightBack(),
                                        new TilterSetpointPosition(TilterConstants.IDLE_POSITION)));

//        operator.rightBumper().whileTrue(new CG_ShootingIntake());

        operator.leftBumper().whileTrue(new TilterHome());
        operator.rightBumper().whileTrue(new CG_ShootingIntake())
                .whileFalse(new TilterSetpointPosition(TilterConstants.IDLE_POSITION));

        // uses these for obtaining new data
//        operator.a().whileTrue(new TilterDashboardPosition());
//        operator.b().whileTrue(new ShooterDashboardSpeed());
//        operator.x().whileTrue(new IndexerKick());


        // these are normal operator buttons
        operator.y().whileTrue(new CG_ShootingSpeaker())
                .whileFalse(new TilterSetpointPosition(TilterConstants.IDLE_POSITION));
        operator.a().whileTrue(new TilterSetpointPosition(0));
        operator.b().whileTrue(new FlipperUp());
        operator.x().whileTrue(new FlipperDown());
//        operator.b().whileTrue(new CG_ShootingAmp())
//                .whileFalse(new ParallelCommandGroup(
//                        new TilterSetpointPosition(TilterConstants.IDLE_POSITION),
//                        new FlipperDown()
//                        ));
//        operator.x().whileTrue(new IndexerKick());
    }

    private void configureButtonBindings() {

//        driver.a().whileTrue(new TilterDashboardPosition());
//        driver.a().whileTrue(new CG_ShootingLime());
//        driver.b().whileTrue(new SwerveRotateLime());
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
        autoChooser.addOption("2 note AMP", AutoBuilder.buildAuto("top 1 + 1"));
        autoChooser.addOption("2 note HUMAN PLAYER", AutoBuilder.buildAuto("bottom 1 + 1"));
        autoChooser.addOption("2 note middle", AutoBuilder.buildAuto("middle 1 + 1"));
        autoChooser.addOption("4 note", AutoBuilder.buildAuto("3 note"));
        autoChooser.addOption("top sprint 5 note", AutoBuilder.buildAuto("top sprint 5 note"));
//        autoChooser.addOption("top sprint 6 note", AutoBuilder.buildAuto("top sprint 6 note"));
        autoChooser.addOption("top run", AutoBuilder.buildAuto("top run"));
        autoChooser.addOption("bottom run", AutoBuilder.buildAuto("bottom run"));
        autoChooser.addOption("bottom sprint", AutoBuilder.buildAuto("bottom sprint"));

        SmartDashboard.putData("Auto Command", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}