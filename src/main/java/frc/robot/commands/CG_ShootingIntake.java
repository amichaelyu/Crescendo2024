package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TilterConstants;
import frc.robot.subsystems.*;

public class CG_ShootingIntake extends ParallelCommandGroup {
    public CG_ShootingIntake(Shooter shooter, Tilter tilter) {
        super(
                new ShooterSetpointSpeed(shooter, ShooterConstants.INTAKE_SPEED),
                new TilterSetpointPosition(tilter, TilterConstants.INTAKE_POSITION)
        );
    }
}