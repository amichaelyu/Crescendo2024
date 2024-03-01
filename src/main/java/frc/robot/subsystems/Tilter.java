package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.TilterConstants;

public class Tilter extends SubsystemBase {
  private final TalonFX m_tilterMotor = new TalonFX(TilterConstants.tilterMotorID);
  private boolean isHomed;
  private double setpoint = 0;
  private DigitalInput limitSwitchTop;
  private final DigitalInput limitSwitchBottom;

  private static Tilter INSTANCE = new Tilter();

  public static Tilter getInstance() { return INSTANCE; }

  /**
   * Creates a new Tilter.
   */
  private Tilter() {
    limitSwitchBottom = new DigitalInput(TilterConstants.kLIFTER_LIMIT_BOTTOM);

    m_tilterMotor.getConfigurator().apply(TilterConstants.talonFXConfigs);
    m_tilterMotor.setPosition(TilterConstants.START_POSITION);

//    m_tilterMotor.setNeutralMode(NeutralModeValue.Brake);

//    m_tilterMotor.setInverted(true);
    isHomed = true;
  }

  @Override
  public void periodic() {
//      setVoltage(SmartDashboard.getNumber("tilter voltage", 0));
    SmartDashboard.putNumber("tilter rotations", m_tilterMotor.getPosition().getValue());

    SmartDashboard.putBoolean("At Bottom", isAtBottom());
    SmartDashboard.putBoolean("tilter at setpoint", atSetpoint());
    if (isAtBottom()) {
      homed();
//      System.out.println("Lifter at Bottom; not going down.");
    }
  }

  public void setVoltage(double voltage) {
      if (isAtBottom() && voltage < 0) {
        m_tilterMotor.setControl(new VoltageOut(0));
        System.out.println("Lifter at Bottom; not going down.");
      }
      else{
        m_tilterMotor.setControl(new VoltageOut(voltage));
      }
    }

  private void resetEncoder() {
    m_tilterMotor.setPosition(0);
  }

  private void homed() {
    isHomed = true;
    resetEncoder();
  }

  public boolean atSetpoint() {
    return (getPosition() > (setpoint - TilterConstants.PID_TOLERANCE)) && (getPosition() < (setpoint + TilterConstants.PID_TOLERANCE));
  }

  public void setPosition(double position) {
    if (isAtBottom()) {

    }
    setpoint = MathUtil.clamp(position, 0, 200);
    if (isHomed) {
      if (m_tilterMotor.getPosition().getValue() > setpoint) {
        m_tilterMotor.setControl(new MotionMagicVoltage(setpoint).withFeedForward(-m_tilterMotor.getPosition().getValue() * 0.01));
      }
      else {
        m_tilterMotor.setControl(new MotionMagicVoltage(setpoint).withFeedForward(m_tilterMotor.getPosition().getValue() * 0.01));
      }
    }
  }

  public double getPosition() {
    return m_tilterMotor.getPosition().getValue();
  }

  public void move(double pwr) {
    if (isAtBottom() && pwr < 0) {
      m_tilterMotor.set(0);
      System.out.println("Lifter at Bottom; not going down.");
    }
    else{
      m_tilterMotor.set(pwr);
    }
  }

 
  public void stop() {
    m_tilterMotor.setControl(new VoltageOut(0));
  }

  public boolean isAtBottom() {
		return !limitSwitchBottom.get();
    }
  }