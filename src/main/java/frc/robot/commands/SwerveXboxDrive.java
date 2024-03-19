package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.controller.BetterXboxController;
import frc.lib.controller.BetterXboxController.Humans;
import frc.robot.Constants.SwerveConstants;
import frc.robot.TunerConstants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;


public class SwerveXboxDrive extends Command {
    private final Swerve s_Swerve = TunerConstants.DriveTrain;
    private final BooleanSupplier robotCentricSup;

    private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
            // deadband is built in to BetterXboxController
//            .withDeadband(TunerConstants.kSpeedAt12VoltsMps * 0.1).withRotationalDeadband(SwerveConstants.maxAngularVelocity * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
            // deadband is built in to BetterXboxController
//            .withDeadband(TunerConstants.kSpeedAt12VoltsMps * 0.1).withRotationalDeadband(SwerveConstants.maxAngularVelocity * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

    public SwerveXboxDrive(BooleanSupplier robotCentricSup) {
        addRequirements(s_Swerve);
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Drive */
        if (!DriverStation.isAutonomous()) {
            if (robotCentricSup.getAsBoolean()) {
                s_Swerve.setControl(
                        driveRobotCentric.withVelocityX(BetterXboxController.getController(Humans.DRIVER).getSwerveTranslation().getX() * TunerConstants.kSpeedAt12VoltsMps)
                                .withVelocityY(BetterXboxController.getController(Humans.DRIVER).getSwerveTranslation().getY() * TunerConstants.kSpeedAt12VoltsMps)
                                .withRotationalRate(BetterXboxController.getController(Humans.DRIVER).getSwerveRotation() * SwerveConstants.maxAngularVelocity)
                );
            }
            else {
                s_Swerve.setControl(
                        driveFieldCentric.withVelocityX(BetterXboxController.getController(Humans.DRIVER).getSwerveTranslation().getX() * TunerConstants.kSpeedAt12VoltsMps)
                                .withVelocityY(BetterXboxController.getController(Humans.DRIVER).getSwerveTranslation().getY() * TunerConstants.kSpeedAt12VoltsMps)
                                .withRotationalRate(BetterXboxController.getController(Humans.DRIVER).getSwerveRotation() * SwerveConstants.maxAngularVelocity)
                );
            }
        }
    }
}