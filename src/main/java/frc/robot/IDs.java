package frc.robot;

public enum IDs {
    // CAN IDs

    FRONT_LEFT_DRIVE(0, "rio"),
    FRONT_LEFT_STEER(1, "rio"),
    FRONT_LEFT_ENCODER(2, "rio"),
    FRONT_RIGHT_DRIVE(3, "rio"),
    FRONT_RIGHT_STEER(4, "rio"),
    FRONT_RIGHT_ENCODER(5, "rio"),
    BACK_LEFT_DRIVE(6, "rio"),
    BACK_LEFT_STEER(7, "rio"),
    BACK_LEFT_ENCODER(8, "rio"),
    BACK_RIGHT_DRIVE(9, "rio"),
    BACK_RIGHT_STEER(10, "rio"),
    BACK_RIGHT_ENCODER(11, "rio"),

    PIGEON(12, "rio"),

    // IDs 0-9 Reserved for drivetrain
    ARM_ELEVATOR_MAIN(13, "rio"),
    ARM_ELEVATOR_FOLLOWER(14, "rio"),
    ARM_WRIST(15, "rio"),
    ARM_END_EFFECTOR(16, "rio"),

    CLIMBER_MAIN(20, "rio"),
    CLIMBER_FOLLOWER(21, "rio"),

    INTAKE_PIVOT(31, "rio"),
    INTAKE_ROLLERS(32, "rio"),

    SHOOTER_HOOD(41, "rio"),
    SHOOTER_FLYWHEEL_MAIN(42, "rio"),
    SHOOTER_FLYWHEEL_FOLLOWER(43, "rio"),
    SHOOTER_FEEDER(44, "rio"),
    
    // Rio Digital Ports
    INTAKE_BEAM_BREAK(0, "RIO_DIGITAL_IN"),
    SHOOTER_FEEDER_BEAM_BREAK(0, "RIO_DIGITAL_IN");

    public final int id;
    public final String bus;

    private IDs(int id, String bus) {
        this.id = id;
        this.bus = bus;
    }
}
