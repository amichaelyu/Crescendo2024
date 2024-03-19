package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TilterConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Tilter;


public class TilterPose extends Command {
//    private final Limelight limelight = Limelight.getInstance();
    private final Tilter tilter = Tilter.getInstance();
//    private double distance;

    public TilterPose() {
        addRequirements(this.tilter);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        tilter.setPosition(TilterConstants.tilterMap.get(Swerve.getInstance().distanceToSpeakerSwervePose()));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        tilter.stop();
    }
}