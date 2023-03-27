package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DuckSwerveDrive;

public class TeleopSwerve extends CommandBase {
    private final DuckSwerveDrive m_swerveSubsystem;
    private final DoubleSupplier joystickMovementX;
    private final DoubleSupplier joystickMovementY;
    private final DoubleSupplier joystickMovementZ;

    public TeleopSwerve(DuckSwerveDrive swerveSubsystem, DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotationZ) {
        this.m_swerveSubsystem = swerveSubsystem;
        this.joystickMovementX = translationX;
        this.joystickMovementY = translationY;
        this.joystickMovementZ = rotationZ;
        addRequirements(this.m_swerveSubsystem);
    }
    
    @Override
    public void execute() {
        double x = MathUtil.applyDeadband(joystickMovementX.getAsDouble(), Constants.DriveControllerCharacteristics.deadband);
        double y = MathUtil.applyDeadband(joystickMovementY.getAsDouble(), Constants.DriveControllerCharacteristics.deadband);
        double z = MathUtil.applyDeadband(joystickMovementZ.getAsDouble(), Constants.DriveControllerCharacteristics.deadband);
        m_swerveSubsystem.driveRobotCentric(x,y, z * 45.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
