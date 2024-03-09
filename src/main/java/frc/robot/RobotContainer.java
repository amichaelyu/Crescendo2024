package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.controller.BetterXboxController;
import frc.lib.controller.BetterXboxController.Humans;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TilterConstants;
import frc.robot.commands.*;
import frc.robot.commands.old.TeleIntake;
import frc.robot.subsystems.*;

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
        SysIdRoutine driveRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(null, Units.Volts.of(4),  null, null),
                new SysIdRoutine.Mechanism((volts) -> Swerve.getInstance().setModuleVoltage(volts.in(Units.Volts)),
                        log -> {
                            log.motor("drive-motor")
                                    .voltage(
                                            m_appliedVoltage.mut_replace(
                                                    Swerve.getInstance().getMotorVoltage(), Volts))
                                    .linearPosition(m_distance.mut_replace(Swerve.getInstance().getMotorPosition(), Meters))
                                    .linearVelocity(
                                            m_velocity.mut_replace(Swerve.getInstance().getMotorVelocity(), MetersPerSecond));
                        }, Swerve.getInstance())
        );

        SysIdRoutine shooterRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(null, Units.Volts.of(4),  null, null),
                new SysIdRoutine.Mechanism((volts) -> Shooter.getInstance().setVoltage(volts.in(Units.Volts)),
                        log -> {
                            log.motor("shooter-motor")
                                    .voltage(
                                            m_appliedVoltage.mut_replace(
                                                    Shooter.getInstance().getVoltage(), Volts))
                                    .linearPosition(m_distance.mut_replace(Shooter.getInstance().getPosition(), Meters))
                                    .linearVelocity(
                                            m_velocity.mut_replace(Shooter.getInstance().getVelocity(), MetersPerSecond));
                        }, Shooter.getInstance())
        );

        SysIdRoutine tilterRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(null, Units.Volts.of(4),  null, null),
                new SysIdRoutine.Mechanism((volts) -> Tilter.getInstance().setVoltage(volts.in(Units.Volts)),
                        log -> {
                            log.motor("tilter-motor")
                                    .voltage(
                                            m_appliedVoltage.mut_replace(
                                                    Tilter.getInstance().getVoltage(), Volts))
                                    .linearPosition(m_distance.mut_replace(Tilter.getInstance().getPosition(), Meters))
                                    .linearVelocity(
                                            m_velocity.mut_replace(Tilter.getInstance().getVelocity(), MetersPerSecond));
                        }, Tilter.getInstance())
        );


        NamedCommands.registerCommand("speakerShot", new CG_ShootingSpeaker());
        NamedCommands.registerCommand("intake", new CG_IntakeIndexer());
        NamedCommands.registerCommand("slightBack", new IndexerSlightBack());
        NamedCommands.registerCommand("citrusShot", new CG_ShootingLime());
        NamedCommands.registerCommand("citrusShotNoRot", new CG_ShootingLimeNoRot());
        NamedCommands.registerCommand("citrusPose", new SwerveLimePose());

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
        Indexer.getInstance().setDefaultCommand(new IndexerManual(() -> (0.25 * (driver.getRightTriggerAxis() - driver.getLeftTriggerAxis()))));
        Tilter.getInstance().setDefaultCommand(new TilterManual(() -> ((operator.getRightTriggerAxis() - operator.getLeftTriggerAxis()))));
        Climber.getInstance().setDefaultCommand(new ClimberManual(() -> (operator.getRawAxis(5))));
    }

    private void competitionButtons() {
        driver.a().whileTrue(new CG_ShootingLime())
                .whileFalse(new TilterSetpointPosition(TilterConstants.IDLE_POSITION));
        driver.b().whileTrue(new CG_ShootingLimeOnTheFly())
                .whileFalse(new TilterSetpointPosition(TilterConstants.IDLE_POSITION));
        driver.x().whileTrue(new CG_FlushShots());
        driver.y().onTrue(new InstantCommand(Swerve.getInstance()::zeroHeading));

        driver.rightBumper().whileTrue(new CG_IntakeIndexer())
                            .whileFalse(
                                    new ParallelCommandGroup(
                                        new IndexerSlightBack(),
                                        new TilterSetpointPosition(TilterConstants.IDLE_POSITION)
                                    ));

//        operator.rightBumper().whileTrue(new CG_ShootingIntake());

        operator.leftBumper().whileTrue(new TilterHome());
        operator.rightBumper().whileTrue(new CG_ShootingIntake())
                .whileFalse(new TilterSetpointPosition(TilterConstants.IDLE_POSITION));

        // uses these for obtaining new data
       operator.a().whileTrue(new TilterDashboardPosition());
        operator.b().whileTrue(new ShooterDashboardSpeed());
        operator.x().whileTrue(new IndexerKick());


        // these are normal operator buttons
       // operator.y().whileTrue(new CG_ShootingSpeaker())
                //.whileFalse(new TilterSetpointPosition(TilterConstants.IDLE_POSITION));
       // operator.b().whileTrue(new CG_ShootingAmp())
                //.whileFalse(new TilterSetpointPosition(TilterConstants.IDLE_POSITION));
       // operator.x().whileTrue(new IndexerKick());
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
        autoChooser.addOption("2 note AMP", AutoBuilder.buildAuto("top 1 + 1"));
        autoChooser.addOption("2 note HUMAN PLAYER", AutoBuilder.buildAuto("bottom 1 + 1"));
        autoChooser.addOption("2 note middle", AutoBuilder.buildAuto("middle 1 + 1"));
//        autoChooser.addOption("3 note", AutoBuilder.buildAuto("3 note"));

        SmartDashboard.putData("Auto Command", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}