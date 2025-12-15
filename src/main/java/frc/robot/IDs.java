package frc.robot;

public enum IDs {
    // IDs 0-9 Reserved for drivetrain
    ARM_ELEVATOR_MAIN(10, "rio"),
    ARM_ELEVATOR_FOLLOWER(11, "rio"),
    ARM_WRIST(12, "rio"),
    ARM_END_EFFECTOR(13, "rio"),
    CLIMBER_MAIN(20, "rio"),
    CLIMBER_FOLLOWER(21, "rio"),
    INTAKE_PIVOT(31, "rio"),
    INTAKE_ROLLERS(32, "rio"),
    SHOOTER_HOOD(41, "rio"),
    SHOOTER_FLYWHEEL_MAIN(42, "rio"),
    SHOOTER_FLYWHEEL_FOLLOWER(43, "rio"),
    SHOOTER_FEEDER(44, "rio");

    public final int id;
    public final String bus;

    private IDs(int id, String bus) {
        this.id = id;
        this.bus = bus;
    }
}
