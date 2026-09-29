package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.Map;

public class RobotContainer extends SubsystemBase {
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();
        // Test
        private final LEDSubsystem m_LEDSubsystem = new LEDSubsystem();

        private final CommandXboxController m_driverController = new CommandXboxController(
                        OIConstants.kDriverControllerPort);

        private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3.0);
        private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3.0);
        private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3.0);

        private boolean fieldRelative = true;

        private final SendableChooser<Command> m_chooser = new SendableChooser<>();
        private final SendableChooser<String> m_alliance = new SendableChooser<>();

        private double limelightDistance;

        public RobotContainer() {
                // new ParallelCommandGroup(q12w3q21wqwqaaq
                // new AutonomousShootCommand(m_robotShooter, m_robotIndexer, 2950,
                // m_driverController)));
                // new CalculateTurretPosition(m_robotTurret, "limelight-main",
                // m_alliance.getSelected())));

                SmartDashboard.putNumber("Flywheel Speed", 10);

                configureButtonBindings();
                m_robotDrive.zeroHeading();

                m_robotDrive.setDefaultCommand(
                                new RunCommand(() -> {
                                        double xInput = MathUtil.applyDeadband(m_driverController.getLeftY(),
                                                        OIConstants.kDriveDeadband);
                                        double yInput = MathUtil.applyDeadband(m_driverController.getLeftX(),
                                                        OIConstants.kDriveDeadband);
                                        double rotInput = MathUtil.applyDeadband(m_driverController.getRightX(),
                                                        OIConstants.kDriveDeadband);

                                        double xSpeed = -m_xspeedLimiter.calculate(xInput)
                                                        * DriveConstants.kMaxSpeedMetersPerSecond;
                                        double ySpeed = -m_yspeedLimiter.calculate(yInput)
                                                        * DriveConstants.kMaxSpeedMetersPerSecond;
                                        double rot = -m_rotLimiter.calculate(rotInput)
                                                        * DriveConstants.kMaxAngularSpeed;

                                        m_robotDrive.drive(xSpeed, ySpeed, rot, fieldRelative, 0);
                                }, m_robotDrive));

                setupShuffleboard();
        }

        private void configureButtonBindings() {
                // ====================
                // Driver Controls (Stone)
                // ====================
                m_driverController.b().onTrue(new InstantCommand(() -> m_robotDrive.ResetGyro())); // Reset Gyro
                m_driverController.povLeft().onTrue(new InstantCommand(() -> fieldRelative = !fieldRelative)); // Toggle
                                                                                                               // Field
                                    
        }

        @Override
        public void periodic() {
                // isLocked = LimelightHelpers.getFiducialID("limelight-main") == 26
                // || LimelightHelpers.getFiducialID("limelight-main") == 21 ||
                // LimelightHelpers.getFiducialID("limelight-main") == 18
                // || LimelightHelpers.getFiducialID("limelight-main") == 5
                // || LimelightHelpers.getFiducialID("limelight-main") == 10
                // || LimelightHelpers.getFiducialID("limelight-main") == 2;

                // turretLocked = isLocked ? "Turret Locked" : "Turret Not Locked";
        }

        // ====================
        // Shuffleboard Control Layout For Drivers (Remove if causes issues)
        // ===================
        private void setupShuffleboard() {
                ShuffleboardTab layoutTab = Shuffleboard.getTab("Control Bindings");

                // new ParallelCommandGroup(
                // new AutonomousShootCommand(m_robotShooter, m_robotIndexer, 2950,
                // m_driverController),
                // new CalculateTurretPosition(m_robotTurret, "limelight-main",
                // m_alliance.getSelected()));

                m_chooser.addOption("Do Nothing", new WaitCommand(20));

                m_alliance.setDefaultOption("Red", "Red");
                m_alliance.addOption("Blue", "Blue");

                var driverLayout = layoutTab.getLayout("Driver (Stone)", BuiltInLayouts.kList)
                                .withSize(2, 4)
                                .withPosition(0, 0);

                driverLayout.addString("B Button", () -> "Reset Gyro");
                driverLayout.addString("Y Button", () -> "Toggle Intake (-1500)");
                driverLayout.addString("A Button", () -> "Hold Outtake (750)");
                driverLayout.addString("POV Left", () -> "Toggle Field Relative");
                driverLayout.addString("Right Bumper", () -> "Limelight Shoot Seq");

                var turretLayout = layoutTab.getLayout("Turret (Josh)", BuiltInLayouts.kList)
                                .withSize(2, 3)
                                .withPosition(2, 0);

                turretLayout.addString("Triggers", () -> "Rotate Turret");
                turretLayout.addString("Right Bumper", () -> "Set X-Pattern");
                turretLayout.addString("X Button", () -> "Auto-Aim (Limelight)");
                turretLayout.addString("Left Bumper", () -> "Manual Shoot Seq");

                layoutTab.add("Alliance", m_alliance)
                                .withPosition(4, 1);
                layoutTab.add("Autonomous", m_chooser)
                                .withPosition(4, 2);

                layoutTab.addBoolean("Field Relative", () -> fieldRelative)
                                .withPosition(4, 0)
                                .withWidget("Boolean Box")
                                .withProperties(Map.of("Color when true", "Green", "Color when false", "Red"));
                layoutTab.addDouble("Gyro Degree", () -> m_robotDrive.getHeading())
                                .withPosition(2, 3)
                                .withWidget("Gyro");

                layoutTab.addDouble("Limelight Distance (Shooter)", () -> limelightDistance)
                                .withPosition(6, 4);
        }

        public Command getAutonomousCommand() {
                return m_chooser.getSelected();
        }
}