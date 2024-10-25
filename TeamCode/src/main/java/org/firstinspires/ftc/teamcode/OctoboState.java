package org.firstinspires.ftc.teamcode;

public class OctoboState {
    private double armPosition = 0;

    public OctoboState()
    {
    }

    public void SaveArmPosition(double armPosition)
    {
        this.armPosition = armPosition;
    }

    public  void SetArmPosition(double armPosition){
        this.armPosition = armPosition;
    }

    public double GetArmPosition(){
        return this.armPosition;
    }
}
