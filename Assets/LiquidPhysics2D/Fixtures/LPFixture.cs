using UnityEngine;
using System.Collections;
using System;
using System.Collections.Generic;
#if UNITY_EDITOR
using UnityEditor;
#endif

/// <summary>
/// Represents a fixture in the liquidfun simulation</summary>
[RequireComponent (typeof (LPBody))]
public abstract class LPFixture : LPCorporeal
{
	[Tooltip("Density of this fixture. Will determine its mass")]
	public float Density = 1f;
	[Tooltip("Is this fixture a sensor? Sensors dont interact in collisions")]
	public bool IsSensor = false;
	[Tooltip("Offset of this fixture from the body")]
	public Vector2 Offset;
	[Tooltip("What material is this made out of? Governs bounciness and friction")]
	public PhysicsMaterial2D PhysMaterial;
	
	public int CollisionGroupIndex = 0;
		
	protected float actualRestitution;
	protected float actualFriction;
	protected float defaultRestitution = 0.2f;
	protected float defaultFriction = 0.1f;
	protected LPShapeTypes Shapetype;
	/// <summary>key of this fixture in the LPbody dictionary myFixtures
	/// and its userdata value in the simulation</summary>	
	protected int myIndex;	
	private LPBody body;
		
	UInt16 categoryBits = 0x0001;
	UInt16 maskBits = 0xFFFF;
	
	#if UNITY_EDITOR
	void Start () 
	{
		if(gameObject.GetComponent<LPBody>() == null)
		{  
			Debug.LogError("There must be a LiquidFunBody component in this gameobject for this fixture to do anything");
		}
	}
	#endif

    /// <summary>Set the collision filter data for this fixture</summary>
    public void SetFixtureFilterData(Int16 groupIndex, UInt16 categoryBits, UInt16 maskBits)
    {
        LPAPIFixture.SetFixtureFilterData(ThingPtr, groupIndex, categoryBits, maskBits);

        if (SubPtrs != null)
        {
            foreach (IntPtr ptr in SubPtrs)
            {
                LPAPIFixture.SetFixtureFilterData(ptr, groupIndex, categoryBits, maskBits);
            }
        }
        
    }
	
	/// <summary>Determine this fixtures physics properties</summary>
	protected void GetPhysProps()
	{
		if (PhysMaterial != null) 
		{
			actualRestitution = PhysMaterial.bounciness;
			actualFriction = PhysMaterial.friction;
		}
		else
		{
			actualRestitution = defaultRestitution;
			actualFriction = defaultFriction;
		}
	}
	/// <summary>Determine what colour to draw this fixtures gizmo</summary>
	protected Color GetColor()
	{	
		#if UNITY_EDITOR
		if (Selection.Contains(gameObject))
		{
		 	return LPColors.Selected;
		}
		else if (IsSensor)
		{
		 	return LPColors.Sensor;
		}
		#endif
		if (body != null) return body.GetColor();
		return GetComponent<LPBody>().GetColor();	

	}	
	/// <summary>Get the shape pointer for this fixture</summary>	
	public abstract IntPtr GetShape();
	
	/// <summary>Create this fixture in the simulation</summary>								
	public virtual void Initialise(LPBody bod)
	{
		body = bod;
		myIndex = body.AddFixture(this);
		GetPhysProps();
		
	    IntPtr shape = GetShape();

		ThingPtr = LPAPIFixture.AddFixture(body.GetPtr(),(int)Shapetype
		                                   ,shape
		                                   ,Density,actualFriction,actualRestitution,IsSensor,myIndex);	
		                                   
		LPAPIFixture.SetFixtureFilterData(ThingPtr,(Int16)CollisionGroupIndex,categoryBits,maskBits);
	}
	
	/// <summary>Create this fixture in the simulation with a shape pointer you already have
	/// Note: Used for concave fixtures or fixtures with more than 8 vertices
	/// They are broken up into several fixtures </summary>	
	protected override void InitialiseWithShape(IntPtr shape)
	{
		IntPtr fix = LPAPIFixture.AddFixture(body.GetPtr(),(int)Shapetype
		                                     ,shape
		                                     ,Density,actualFriction,actualRestitution,IsSensor,myIndex);
		                                     
		LPAPIFixture.SetFixtureFilterData(fix,(Int16)CollisionGroupIndex,categoryBits,maskBits);    
		                               
		SubPtrs.Add(fix);	
	}
	
	/// <summary>If the shape you attempt to make is complex log an error</summary>	
	protected override void LogComplex()
	{
		Debug.LogError("Polygon fixture # "+myIndex+" on Body # "+body.myIndex+
		               " is complex! ie. has self intersecting edges. Creating default shape instead");
		
	}
	
	/// <summary>Delete this fixture. In the simulation and in unity</summary>	
	public override void Delete ()
	{
		body.RemoveFixture(myIndex);
		LPAPIFixture.DeleteFixture(body.GetPtr(),ThingPtr);
		if (SubPtrs != null)
		{
			foreach (IntPtr fix in SubPtrs)
			{
				LPAPIFixture.DeleteFixture(body.GetPtr(),fix);
            }	
		}

		Destroy(this);	
	}
}
