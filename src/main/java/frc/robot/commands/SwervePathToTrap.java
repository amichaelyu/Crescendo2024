package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;


public class SwervePathToTrap extends Command {
    private final Swerve swerve = Swerve.getInstance();

    public SwervePathToTrap() {
        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (DriverStation.getAlliance().isPresent()) {
//            AutoBuilder.pathfindToPose(FieldConstants.allianceFlipper(AutoConstants.TRAP_POSITION, DriverStation.getAlliance().get()), new PathConstraints(SwerveConstants.AUTO_MAX_SPEED, , SwerveConstants.maxAngularVelocity))
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}