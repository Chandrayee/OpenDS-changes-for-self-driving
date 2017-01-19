package eu.opends.car;

import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;

public class test {

	public static void main(String[] args) 
	{
		Quaternion rotation = new Quaternion(0f,.7024f,0f,.7118f);
		float[] angles = rotation.toAngles(null);
		angles[1] = -angles[1];
		rotation = new Quaternion().fromAngles(angles);
		System.out.println(rotation.getX()+", "+rotation.getY()+", "+rotation.getZ()+", "+rotation.getZ());
		System.out.println(angles[0]+", "+angles[1]+", "+angles[2]);
		System.out.println(angles[1] * 180/FastMath.PI);
	}

}
