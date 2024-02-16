package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake extends SubsystemBase {
  CANSparkMax m_intakeMotor;

  /** Creates a new Intake. */
  public Intake() {
    m_intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorID, MotorType.kBrushless);

    m_intakeMotor.enableVoltageCompensation(12.0);
    m_intakeMotor.setInverted(true);
  }

  
  public void move(double pwr) {
    m_intakeMotor.set(pwr);
  }


 
  public void stop() {
    m_intakeMotor.stopMotor();
  }

 
}