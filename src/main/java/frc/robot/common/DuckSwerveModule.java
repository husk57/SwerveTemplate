package frc.robot.common;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Constants;

public class DuckSwerveModule {
    private final CANSparkMax m_turnMotor;
    private double angleGoal;
    private final WPI_TalonFX m_driveMotor;
    private final AnalogEncoder absoluteEncoder;
    
    private final RelativeEncoder m_turnEncoder;

    //PF controller (supply static ff as plant)
    private final SparkMaxPIDController m_turnController;
    private final double KS_VOLTS_TURNING;

    public DuckSwerveModule(int CAN_ID_TURN_MOTOR, int CAN_ID_DRIVE_MOTOR, int ANALOG_PWM_PORT_NUMBER, double KS_VOLTS_TURNING, double ENCODER_OFFSET) {
        //motor configuration on the burned flash, change via dashboard
        this.m_turnMotor = new CANSparkMax(CAN_ID_TURN_MOTOR, MotorType.kBrushless);
        this.m_driveMotor = new WPI_TalonFX(CAN_ID_DRIVE_MOTOR);

        this.m_turnEncoder = m_turnMotor.getEncoder(); //the motor might need to be inverted if it doesnt rotate CCW+
        this.m_turnController = m_turnMotor.getPIDController();
        this.m_turnController.setFeedbackDevice(m_turnEncoder);
        
        this.KS_VOLTS_TURNING = KS_VOLTS_TURNING;

        //falcon config
        this.m_driveMotor.setNeutralMode(NeutralMode.Brake);

        this.absoluteEncoder = new AnalogEncoder(ANALOG_PWM_PORT_NUMBER);
        //in terms of revolutions on the pinion
        this.m_turnEncoder.setPosition(this.absoluteEncoder.getAbsolutePosition()*Constants.SwerveDriveCharacteristics.azimuthGearing);
        this.angleGoal = ENCODER_OFFSET*360.0; //should want to move to here
    }

    public void setTargetAngle(double angle) {
        angleGoal = angle;
    }
    public void applyControlAlgorithms() {
        m_turnController.setReference((angleGoal/360.0)*Constants.SwerveDriveCharacteristics.azimuthGearing, ControlType.kPosition, 0, this.KS_VOLTS_TURNING);
    }
    public void setTurnSpeed(double speed) {
        m_turnMotor.set(speed);
    }
    public void setDriveSpeed(double speed) {
        m_driveMotor.set(speed);
    }
    public double getAnalogEncoderPositionDegrees() {
        return absoluteEncoder.getAbsolutePosition()*360.0;
    }
    public double getAngleGoal() {
        return angleGoal;
    }
}
