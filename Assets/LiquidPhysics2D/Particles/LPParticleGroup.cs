using UnityEngine;
using System.Collections;
using System;
using System.Runtime.InteropServices;

#if UNITY_EDITOR
using UnityEditor;
#endif

/// <summary>
/// Represents particle group in the liquidfun simulation</summary>
public abstract class LPParticleGroup : LPCorporeal 
{	
	[Tooltip("The partcle physics material for this particle group. Drag a LPparticleMaterial scriptable object in here")]
	public LPParticleMaterial ParticlesMaterial;
	[Tooltip("The particle group physics material for this particle group. Drag a LPparticleGroupMaterial scriptable object in here")]
	public LPParticleGroupMaterial GroupMaterial;
	[Tooltip("Color of all the particles created in this group")]
	public Color _Color =  LPColors.DefaultParticleCol;
	[Tooltip("This particle group will be created in the particle system with this index")]
	public int ParticleSystemImIn = 0;
	[Tooltip("Strenght of forces between the particles. Affected by the particles material flags, eg. elastic")]	
	public float Strenght = 1f;
	[Tooltip("Angular velocity this particle group should be created with")]
	public float AngularVelocity = 0f;
	[Tooltip("Linear velocity this particle group should be created with")]
	public Vector2 LinearVelocity = Vector2.zero;
	[Tooltip("Lifetime particles should start with. They will be deleted when their lifetime runs out. Value of 0 indicates infinite lifetime")]
	public float LifeTime = 0.0f;
	[Tooltip("What distance to space between the particles when creating the group. Value of 0 means distance will be the same as particle diamater")]
	public float Stride = 0;
	[Tooltip("Should this particle group be created when the game play?")]
	public bool SpawnOnPlay = true;
	[Tooltip("Userdata value for particles in this group. Use this to denote gameplay properties of particles. eg acid, lava")]
	public int UserData = 0;
	
	private LPParticleSystem sys;
	
	#if UNITY_EDITOR
	void Start ()
	{
		if(GameObject.FindObjectOfType<LPManager>() == null)
		{
			Debug.LogError("There is no LiquidFunManager. You must have one in your scene for Liquid Physics 2D to work");
		}
		if(GameObject.FindObjectOfType<LPParticleSystem>() == null)
		{
			Debug.LogError("There is no ParticleSystem. You must have one in your scene particles in Liquid Physics 2D to work");
		}		
	}
	#endif
	
	/// <summary>
	/// Get the int representing this particles material flags.</summary>
	private int getPartNum()
	{
		Int32 partnum = 0;
		if (ParticlesMaterial != null)
		{
			partnum = ParticlesMaterial.GetInt();
		}
		return partnum;
	}
	
	/// <summary>
	/// Get the int representing this particles group material flags.</summary>
	private int getGroupNum()
	{		
		Int32 groupnum = 0;
		if (GroupMaterial != null)
		{
			groupnum = GroupMaterial.GetInt();
		}
		return groupnum;
	}
	
	/// <summary>If the shape you attempt to make is complex log an error</summary>	
	protected override void LogComplex()
	{
		Debug.LogError("Particle Group # ? is complex! ie. has self intersecting edges. Creating default shape instead"); //HACK		
	}
	
	/// <summary>Create this particle group in the simulation</summary>	
	public void Initialise(LPParticleSystem s)
	{
		sys = s;
		IntPtr shape = GetShape();
		ThingPtr = LPAPIParticleGroups.CreateParticleGroup(sys.GetPtr(),getPartNum(),getGroupNum(),0f,Strenght,AngularVelocity
		                                                   ,LinearVelocity.x,LinearVelocity.y
		                                                   ,shape
		                                                   ,(int)(_Color.r*255f),(int)(_Color.g*255f),(int)(_Color.b*255f),(int)(_Color.a*255f)
            
                                                                                                          ,Stride,LifeTime,UserData);
        LPAPIUtility.ReleaseShape(shape);              
                                                                                            
		if (SubPtrs !=null && ParticlesMaterial !=null && (ParticlesMaterial.elastic || ParticlesMaterial.spring))
		{
			foreach (IntPtr groupptr in SubPtrs)
			{
				LPAPIParticleGroups.JoinParticleGroups(sys.GetPtr(),ThingPtr,groupptr);
			}
		}
	}
	
	/// <summary>Create this particle group in the simulation with a shape pointer you already have
	/// Note: Used for concave fixtures or fixtures with more than 8 vertices
	/// They are broken up into several fixtures </summary>	
	protected override void InitialiseWithShape(IntPtr shape)
	{
		SubPtrs.Add( LPAPIParticleGroups.CreateParticleGroup(sys.GetPtr(),getPartNum(),getGroupNum(),0f,Strenght,AngularVelocity
		                                               ,LinearVelocity.x,LinearVelocity.y
		                                               ,shape
		                                               ,(int)(_Color.r*255f),(int)(_Color.g*255f),(int)(_Color.b*255f),(int)(_Color.a*255f)
		                                               ,Stride,LifeTime,UserData)
		             );	
	}
	
	/// <summary>Delete this particle group. In the simulation and in unity</summary>	
	public override void Delete()
	{
		LPAPIParticleGroups.DeleteParticlesInGroup(ThingPtr);
		Destroy(this);	
	}

    /// <summary>Get the velocity of particle group</summary>
    public Vector2 GetVelocity()
    {
        IntPtr particlesPointer = LPAPIParticleGroups.GetParticleGroupVelocity(ThingPtr);

        float[] particlesArray = new float[2];
        Marshal.Copy(particlesPointer, particlesArray, 0, 2);

        return new Vector2(particlesArray[0], particlesArray[1]);
    }

    /// <summary>Get the position of particle group Note: only works with solid groups</summary>
    public Vector2 GetPosition()
    {
        IntPtr particlesPointer = LPAPIParticleGroups.GetParticleGroupPosition(ThingPtr);

        float[] particlesArray = new float[2];
        Marshal.Copy(particlesPointer, particlesArray, 0, 2);

        return new Vector2(particlesArray[0], particlesArray[1]);
    }
	
	/// <summary>Determine what colour to draw this particle groups gizmo</summary>
	protected Color GetColors()
	{	
		#if UNITY_EDITOR
		if (Selection.Contains(gameObject)) 
		{
			return LPColors.Selected;
        }
        #endif
        return LPColors.ParticleGroup;  
    }

	protected abstract IntPtr GetShape();
	
}
