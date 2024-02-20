package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.controller.BetterXboxController;
import frc.lib.controller.BetterXboxController.Humans;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;


public class TeleopSwerve extends Command {    
    private final Swerve s_Swerve = Swerve.getInstance();
    private final BooleanSupplier robotCentricSup;

    public TeleopSwerve(BooleanSupplier robotCentricSup) {
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