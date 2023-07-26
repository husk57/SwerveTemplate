package frc.robot.common;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

public class DuckSwerveModule {
    private final CANSparkMax m_turnMotor;
    private double angleGoal;
    private final WPI_TalonFX m_driveMotor;
    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderIsInverted;
    
    private final RelativeEncoder m_turnEncoder;

    //PF controller (supply static ff as plant)
    private final SparkMaxPIDController m_turnController;
    private final double KS_VOLTS_TURNING;

    public DuckSwerveModule(int CAN_ID_TURN_MOTOR, int CAN_ID_DRIVE_MOTOR, int ANALOG_PWM_PORT_NUMBER, double KS_VOLTS_TURNING, boolean absoluteEncoderInverted) {
        //motor configuration on the burned flash, change via dashboard
        this.m_turnMotor = new CANSparkMax(CAN_ID_TURN_MOTOR, MotorType.kBrushless);
        this.m_driveMotor = new WPI_TalonFX(CAN_ID_DRIVE_MOTOR);

        this.m_turnEncoder = m_turnMotor.getEncoder(); //the motor might need to be inverted if it doesnt rotate CCW+
        this.m_turnController = m_turnMotor.getPIDController();
        this.m_turnController.setFeedbackDevice(m_turnEncoder);
        
        this.KS_VOLTS_TURNING = KS_VOLTS_TURNING;
        
        this.absoluteEncoderIsInverted = absoluteEncoderInverted;
        this.absoluteEncoder = new AnalogInput(ANALOG_PWM_PORT_NUMBER);
        this.m_turnEncoder.setPosition(0);
        this.angleGoal = 0; //should be the same as the home angle
    }

    public void setTargetAngle(double angle) {
        angleGoal = angle;
    }
    public void applyControlAlgorithms() {
        m_turnController.setReference(angleGoal/360.0, ControlType.kPosition, 0, this.KS_VOLTS_TURNING);
    }
    public void setTurnSpeed(double speed) {
        m_turnMotor.set(speed);
    }
    public void setDriveSpeed(double speed) {
        m_driveMotor.set(speed);
    }
    public double getAnalogEncoderPosition() {
        return (absoluteEncoderIsInverted ? -1.0 : 1.0) * ((absoluteEncoder.getAverageVoltage() / RobotController.getVoltage5V()) * (Math.PI * 2) - Math.PI);
    }
}
