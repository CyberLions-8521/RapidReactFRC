package frc.robot.commands.LimelightShooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.lang.Math;
import java.time.Year;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Limelight;

/** An example command that uses an example subsystem. */
public class CalculateDistance extends CommandBase {
    private final Limelight m_kVision;
    private double m_Y;

    public CalculateDistance(Limelight vision) {
        m_kVision = vision;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(vision);
    }

    public double distance;

    // Medium Filter
    LinearFilter filter = LinearFilter.movingAverage(10);

    // Math Equation
    public void Distance() {
        m_Y = Math.toRadians(filter.calculate(m_kVision.getTy()));
        distance = ((VisionConstants.HeightOfTarget - VisionConstants.HeightOfCamera)
                / Math.tan(VisionConstants.CameraAngle + m_Y));

    }

    @Override
    public void initialize() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Distance();
        SmartDashboard.putNumber("Distance", getDistance());

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    public double getDistance() {

        return distance;
    }

}