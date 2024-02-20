package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TilterConstants;

public class CG_ShootingIntake extends ParallelCommandGroup {
    public CG_ShootingIntake() {
        super(
                new ShooterSetpointSpeed(ShooterConstants.INTAKE_SPEED),
                new TilterSetpointPosition(TilterConstants.INTAKE_POSITION)
        );
    }
}