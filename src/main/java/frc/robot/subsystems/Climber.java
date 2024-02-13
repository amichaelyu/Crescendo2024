package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Climber extends SubsystemBase {
  CANSparkMax m_rightClimberMotor;
  CANSparkMax m_leftClimberMotor;

  /** Creates a new Climber. */
  public Climber() {
    

    m_rightClimberMotor=new CANSparkMax(rightClimberMotorID, MotorType.kBrushless);
    m_leftClimberMotor=new CANSparkMax(leftClimberMotorID, MotorType.kBrushless);

    m_rightClimberMotor.setInverted(true);
    m_leftClimberMotor.setInverted(true);

    m_rightClimberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_leftClimberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    
  }

  
  public void move(double pwr) {
   
    m_rightClimberMotor.set(pwr);
    m_leftClimberMotor.set(pwr);

  }


 
  public void stop() {
    m_rightClimberMotor.stopMotor();
    m_leftClimberMotor.stopMotor();
  }

 
}