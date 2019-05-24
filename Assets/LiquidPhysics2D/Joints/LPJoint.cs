using UnityEngine;
using System;

public abstract class LPJoint : LPThing 
{
	[Tooltip("Gameobject with 1st body attached to this joint")]
	public GameObject BodyA;
	[Tooltip("Gameobject with 2nd body attached to this joint")]
	public GameObject BodyB;
	[Tooltip("Should the bodies connected by this joint collide with each other?")]
	public bool CollideConnected = false;
	
	protected LPManager lpman;
	
	/// <summary>Create this joint in the simulation</summary>	
	public void Initialise(LPManager man)
	{
		if (BodyA !=null && BodyB !=null && BodyA.GetComponent<LPBody>() != null && BodyB.GetComponent<LPBody>() != null)
		{
			lpman = man;
			Initialise2(lpman.GetPtr());
		}
		else Debug.LogError("This Joint must be assigned 2 Bodies to connect in order to be created");
	}
	
	protected abstract void Initialise2(IntPtr world);
	
	/// <summary>Delete this joint, in the simulation and in unity</summary>	
	public override void Delete()
	{
		LPAPIJoint.DeleteJoint(lpman.GetPtr(),ThingPtr);
		Destroy(this);
	}
}
