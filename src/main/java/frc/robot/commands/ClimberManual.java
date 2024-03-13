package frc.robot.commands;



import frc.robot.subsystems.Climber;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Tilter;

import java.util.function.DoubleSupplier;


public class ClimberManual extends Command {
  private final Climber climber = Climber.getInstance();
  private final Tilter tilter = Tilter.getInstance();
  private final DoubleSupplier pwr;

  public ClimberManual(DoubleSupplier pwr) {
      this.pwr = pwr;
      addRequirements(climber, tilter);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void initialize() {
  }

   // Called every time the scheduler runs while the command is scheduled.
   @Override
   public void execute() {
      if (MathUtil.applyDeadband(pwr.getAsDouble(),0.1) > 0.2) {
          tilter.setPosition(0);
      }
      climber.move(MathUtil.applyDeadband(pwr.getAsDouble(),0.1));
   }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }
}