package org.bitbuckets;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.bitbuckets.commands.CircleCommand;

import static org.bitbuckets.DriveSubsystem.ks;
import static org.bitbuckets.DriveSubsystem.kv;

public class Robot extends TimedRobot {


    DriveSubsystem driveSubsystem;


    @Override
    public void robotInit() {
        super.robotInit();
        WPI_TalonFX talonFX1 = new WPI_TalonFX(1);
        talonFX1.setInverted(true);
        WPI_TalonFX talonFX2 = new WPI_TalonFX(2);
        XboxController joystick = new XboxController(0);
        DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.55);
        SimpleMotorFeedforward ff = new SimpleMotorFeedforward(ks,kv,0.97626);

       driveSubsystem =  new DriveSubsystem(talonFX1, talonFX2, ff, kinematics, joystick);



    }

    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().schedule(new CircleCommand(driveSubsystem));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopPeriodic() {
        driveSubsystem.ourTeleopPeriod();
    }



}