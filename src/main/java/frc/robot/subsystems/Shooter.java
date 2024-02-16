package frc.robot.subsystems;

import static frc.robot.Constants.*;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.GenericHID;


public class Shooter extends SubsystemBase {
  private final TalonFX m_rightShooter = new TalonFX(ShooterConstants.rightShooterID);
  private final TalonFX m_leftShooter = new TalonFX(ShooterConstants.leftShooterID);

  public Shooter() {
    m_rightShooter.getConfigurator().apply(ShooterConstants.talonFXConfigs);
    m_rightShooter.setInverted(true);
    m_leftShooter.setControl(new Follower(ShooterConstants.rightShooterID, true));
  }

  public void setSpeed(double speed) {
    m_rightShooter.setControl(new MotionMagicVelocityVoltage(speed));
  }

  public void setVoltage(double volts) {
    m_rightShooter.setControl(new VoltageOut(volts));
  }
 
  public void stop() {
    m_rightShooter.stopMotor();
    m_leftShooter.stopMotor();
  }
}