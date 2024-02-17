package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.controller.BetterXboxController;
import frc.lib.controller.BetterXboxController.Humans;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private final Swerve s_Swerve;
    private final BooleanSupplier robotCentricSup;

    public TeleopSwerve(Swerve s_Swerve,  BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Drive */
        if (!DriverStation.isAutonomous()) {
            s_Swerve.drive(
                    BetterXboxController.getController(Humans.DRIVER).getSwerveTranslation().times(SwerveConstants.maxSpeed),
                    BetterXboxController.getController(Humans.DRIVER).getSwerveRotation() * SwerveConstants.maxAngularVelocity,
                    !robotCentricSup.getAsBoolean(),
                    true
            );
        }
    }
}