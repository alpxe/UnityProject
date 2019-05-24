using UnityEngine;
using System.Collections;

public class VelocityDrawer : LPDrawParticleSystem
{
	public float Velscale = 1f;
	
	public override void UpdateParticles(LPParticle[] partdata)
	{	
		if (GetComponent<ParticleEmitter>().particleCount < partdata.Length) 
		{
			GetComponent<ParticleEmitter>().Emit(partdata.Length - GetComponent<ParticleEmitter>().particleCount);		
			particles = GetComponent<ParticleEmitter>().particles;
		}
		
		for (int i=0; i < particles.Length; i ++)
		{		
			if (i > partdata.Length-1)
			{
				particles[i].energy = 0f;
			}
			else
			{
				particles[i].position  = partdata[i].Position;
				particles[i].color = new Color(0.5f+(partdata[i].Velocity.x*0.3f),(partdata[i].Velocity.y*0.3f),0.5f);
			}		
		}
		
		GetComponent<ParticleEmitter>().particles = particles;
	}
}
