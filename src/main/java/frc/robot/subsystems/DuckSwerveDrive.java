package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.DuckSwerveModule;

public class DuckSwerveDrive extends SubsystemBase {
    private final ADXRS450_Gyro gyro;
    private final DuckSwerveModule m_FLModule;
    double currentAngle = 0.0;

    public DuckSwerveDrive() {
        // turn can id, drive can id, absolute encoder can id
        m_FLModule = new DuckSwerveModule(8, 1, 0, 0.05, false);
        gyro = new ADXRS450_Gyro();
    }

    public void driveRobotCentric(double velocityXMetersPerSecond, double velocityYMetersPerSecond,
            double angVelocityDegreesPerSecond) {
                m_FLModule.setTurnSpeed(velocityXMetersPerSecond);
                m_FLModule.setDriveSpeed(velocityYMetersPerSecond);
                System.out.println(m_FLModule.getAnalogEncoderPosition());
    }

    @Override
    public void periodic() {
        //System.out.println(currentAngle);
        //m_FLModule.applyControlAlgorithms();
    }
}
