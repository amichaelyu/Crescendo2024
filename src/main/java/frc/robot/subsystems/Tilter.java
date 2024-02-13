package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Tilter extends SubsystemBase {
  CANSparkMax m_tilterMotor;

  /** Creates a new Intake. */
  public Tilter() {
    

    m_tilterMotor=new CANSparkMax(intakeMotorID, MotorType.kBrushless);

    m_tilterMotor.setInverted(true);
    
  }

  
  public void move(double pwr) {
   
    m_tilterMotor.set(pwr);

  }


 
  public void stop() {
    m_tilterMotor.stopMotor();;
  }

 
}