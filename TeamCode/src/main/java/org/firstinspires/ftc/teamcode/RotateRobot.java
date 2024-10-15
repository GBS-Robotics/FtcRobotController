package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class RotateRobot
{
    float angle;
    VectorF relativeField;
    GeneralMatrixF rotationMatrix = new GeneralMatrixF(3,3);


    public RotateRobot(float yaw, VectorF relativePosition)
    {
        angle = yaw;
        relativeField = relativePosition;

        rotationMatrix.put(1,1,(float) Math.cos(angle));
        rotationMatrix.put(2,1,(float) Math.sin(angle));
        rotationMatrix.put(3,1,(float) 0);

        rotationMatrix.put(1,2,(float) -Math.sin(angle));
        rotationMatrix.put(2,2,(float) Math.cos(angle));
        rotationMatrix.put(3,2,(float) 0);

        rotationMatrix.put(1,3,(float) 0);
        rotationMatrix.put(2,3,(float) 0);
        rotationMatrix.put(3,3,(float) 1);
    }

    public void updateInfo(float yaw, VectorF relativePosition)
    {
        angle = yaw;
        relativeField = relativePosition;

        rotationMatrix.put(1,1,(float) Math.cos(angle));
        rotationMatrix.put(2,1,(float) Math.sin(angle));
        rotationMatrix.put(3,1,(float) 0);

        rotationMatrix.put(1,2,(float) -Math.sin(angle));
        rotationMatrix.put(2,2,(float) Math.cos(angle));
        rotationMatrix.put(3,2,(float) 0);

        rotationMatrix.put(1,3,(float) 0);
        rotationMatrix.put(2,3,(float) 0);
        rotationMatrix.put(3,3,(float) 1);
    }

    public VectorF getRelativeField()
    {
        return rotationMatrix.multiplied(relativeField);
    }
}
