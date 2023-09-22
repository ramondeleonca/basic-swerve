package org.team3526.lib;

public class SwerveOptions {
    // Turn Encoder options
    private double turningEncoderOffsetDeg;
    private boolean turningEncoderinverted;

    // CANCoder ID
    private int turningEncoderID;

    // Motor IDs
    private int driveMotorID;
    private int turningMotorID;

    // Wether to invert the motors or not
    private boolean driveMotorInverted;
    private boolean turningMotorInverted;

    // A name for the Swerve Module
    String name;

    // Constructor
    public SwerveOptions(
        double turnEncoderOffsetDeg,
        boolean turnEncoderinverted,

        int turningEncoderID,

        int driveMotorID,
        int turningMotorID,

        boolean driveMotorInverted,
        boolean turningMotorInverted,

        String name
    ) {
        this.turningEncoderOffsetDeg = turnEncoderOffsetDeg;
        this.turningEncoderinverted = turnEncoderinverted;

        this.turningEncoderID = turningEncoderID;

        this.driveMotorID = driveMotorID;
        this.turningMotorID = turningMotorID;

        this.driveMotorInverted = driveMotorInverted;
        this.turningMotorInverted = turningMotorInverted;

        this.name = name;
    }

    // Getters
    public double getTurningEncoderOffsetDeg() {
        return turningEncoderOffsetDeg;
    }

    public double getTurningEncoderOffsetRad() {
        return Math.toRadians(turningEncoderOffsetDeg);
    }

    public boolean getTurningEncoderInverted() {
        return turningEncoderinverted;
    }

    public int getTurningEncoderID() {
        return turningEncoderID;
    }

    public int getDriveMotorID() {
        return driveMotorID;
    }

    public int getTurningMotorID() {
        return turningMotorID;
    }

    public boolean getDriveMotorInverted() {
        return driveMotorInverted;
    }

    public boolean getTurningMotorInverted() {
        return turningMotorInverted;
    }

    public String getName() {
        return name;
    }

    // Setters
    public void setTurningEncoderOffsetDeg(double turnEncoderOffsetDeg) {
        this.turningEncoderOffsetDeg = turnEncoderOffsetDeg;
    }

    public void setTurnEncoderOffsetRad(double turnEncoderOffsetRad) {
        this.turningEncoderOffsetDeg = Math.toDegrees(turnEncoderOffsetRad);
    }

    public void setTurnEncoderInverted(boolean turnEncoderinverted) {
        this.turningEncoderinverted = turnEncoderinverted;
    }

    public void setTurningEncoderID(int turnEncoderID) {
        this.turningEncoderID = turnEncoderID;
    }

    public void setDriveMotorID(int driveMotorID) {
        this.driveMotorID = driveMotorID;
    }

    public void setTurningMotorID(int turningMotorID) {
        this.turningMotorID = turningMotorID;
    }

    public void setDriveMotorInverted(boolean driveMotorInverted) {
        this.driveMotorInverted = driveMotorInverted;
    }

    public void setTurningMotorInverted(boolean turningMotorInverted) {
        this.turningMotorInverted = turningMotorInverted;
    }

    public void setName(String name) {
        this.name = name;
    }
}
