// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ElevatorOutput;
import frc.robot.Constants.XBOX;

public class Climber extends SubsystemBase {
    public Climber() {
    }

    // private Joystick m_stick;
    // creating encoder
    private RelativeEncoder m_encoder;
    // creating the motor
    public CANSparkMax ElevatorMotor = new CANSparkMax(CAN.kElevator, MotorType.kBrushless);

    // Initilizin the motor
    public void initializeEncoder() {
        // using the encoder to get the value from the motor
        m_encoder = ElevatorMotor.getEncoder();
        // default position = 0
        m_encoder.setPosition(0);
    }

    // setting up the controller
    public void Tarzan(XboxController controller) {
        m_encoder = ElevatorMotor.getEncoder();
        // if you hit the dpad right and the position is less than x , make it go in one
        // direction
        if (controller.getPOV() == 90 && m_encoder.getPosition() < 100) {
            ElevatorMotor.setVoltage(ElevatorOutput.ElevatorUp);
            // if you hit the dpad left and the position is greater than 0, make the motor
            // go in the other direction
        } else if (controller.getPOV() == 270 && m_encoder.getPosition() > 0) {
            ElevatorMotor.setVoltage(ElevatorOutput.ElevatorDown);
            // if you are NOT pressing the dpap button or the position is greater than x or
            // the position is less than 0 then the motor does nothing
        } else if (controller.getPOV() == -1 || m_encoder.getPosition() > 100 || m_encoder.getPosition() < 0) {
            ElevatorMotor.setVoltage(0);
        }
    }

    @Override
    public void periodic() {
        // we initliaze the motor again.
        m_encoder = ElevatorMotor.getEncoder();
        // we display the position and velocity of the encoder on the shuffleBoard
        SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());
        SmartDashboard.putNumber("Encoder Velocity", m_encoder.getVelocity());
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}