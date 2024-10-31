package org.firstinspires.ftc.teamcode;

public class OctoboState {
    private double armPosition = 0;

    private double wormGearPosition = 0;

    private boolean armLocked = true;

    public OctoboState()
    {
    }

    public void setArmPosition(double armPosition) {
        this.armPosition = armPosition;
    }

    public double getArmPosition() {
        return this.armPosition;
    }

    public double getWormGearPosition() {
        return wormGearPosition;
    }

    public void setWormGearPosition(double wormGearPosition) {
        this.wormGearPosition = wormGearPosition;
    }

    public boolean getArmLocked() {
        return armLocked;
    }

    public void setArmLocked(boolean armLocked) {
        this.armLocked = armLocked;
    }
}
