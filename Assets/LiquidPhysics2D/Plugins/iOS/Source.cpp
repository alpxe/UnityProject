#include "Box2D.h"
#include "utility"
#include "b2Math.h"
#include "math.h"
#include "vector"

class b2NewRaycastCallback : public b2RayCastCallback {
public:
    b2NewRaycastCallback(int m, bool shouldQ) : numFixtures(0), numParticles(0), mode(m), shouldQuery(shouldQ) {}
    virtual float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point,
                                  const b2Vec2& normal, float32 fraction) {
        ++numFixtures;
        fixturesArray.push_back((int64)fixture->GetBody()->GetUserData());
        fixturesArray.push_back((int64)fixture->GetUserData());
        fixturesArray.push_back(point.x);
        fixturesArray.push_back(point.y);
        fixturesArray.push_back(normal.x);
        fixturesArray.push_back(normal.y);
        fixturesArray.push_back(fraction);
        return mode;
    }
    virtual float32 ReportParticle(const b2ParticleSystem* particleSystem,
                                   int32 index, const b2Vec2& point,
                                   const b2Vec2& normal, float32 fraction) {
        ++numParticles;
        particleSystem->GetUserDataBuffer ();
        particlesArray.push_back(particleSystem->MyIndex);
        particlesArray.push_back(index);
        particlesArray.push_back(point.x);
        particlesArray.push_back(point.y);
        particlesArray.push_back(normal.x);
        particlesArray.push_back(normal.y);
        particlesArray.push_back(fraction);
        return mode;
    }
    virtual bool ShouldQueryParticleSystem(const b2ParticleSystem* particleSystem) {
        return shouldQuery;
    }
    float* GetData() {
        lengthsArray.push_back(numFixtures);
        lengthsArray.push_back(numParticles);
        returnArray.reserve(lengthsArray.size() + fixturesArray.size() + particlesArray.size());
        returnArray.insert(returnArray.end(), lengthsArray.begin(), lengthsArray.end());
        returnArray.insert(returnArray.end(), fixturesArray.begin(), fixturesArray.end());
        returnArray.insert(returnArray.end(), particlesArray.begin(), particlesArray.end());
        positionArray = &returnArray[0];
        return positionArray;
    }
private:
    float* positionArray;
    int numFixtures;
    int numParticles;
    int mode;
    bool shouldQuery;
    std::vector<float> fixturesArray;
    std::vector<float> particlesArray;
    std::vector<float> lengthsArray;
    std::vector<float> returnArray;
};

class b2NewContactListener : public b2ContactListener {
public:
    b2NewContactListener() : fixtureContacts(0), mixedContacts(0), particleContacts(0) {}
    virtual void BeginContact(b2Contact* contact) {
        //if (contact->IsTouching()) {
        ++fixtureContacts;
        b2WorldManifold* m = new b2WorldManifold();
        contact->GetWorldManifold(m);
        fixturesArray.push_back((int64)contact->GetFixtureA()->GetBody()->GetUserData());
        fixturesArray.push_back((int64)contact->GetFixtureB()->GetBody()->GetUserData());
        fixturesArray.push_back((int64)contact->GetFixtureA()->GetUserData());
        fixturesArray.push_back((int64)contact->GetFixtureB()->GetUserData());
        fixturesArray.push_back(m->points[0].x);
        fixturesArray.push_back(m->points[0].y);
        fixturesArray.push_back(m->points[1].x);
        fixturesArray.push_back(m->points[1].y);
        fixturesArray.push_back(m->normal.x);
        fixturesArray.push_back(m->normal.y);
        if (contact->IsTouching()) fixturesArray.push_back(1.0f);
        else fixturesArray.push_back(0.0f);
        
        delete m;
        //}
    }
    virtual void BeginContact(b2ParticleSystem* particleSystem,
                              b2ParticleBodyContact* particleBodyContact) {
        ++mixedContacts;
        mixedArray.push_back(particleSystem->MyIndex);
        mixedArray.push_back(particleBodyContact->index);
        mixedArray.push_back((int64)particleBodyContact->body->GetUserData());
        mixedArray.push_back((int64)particleBodyContact->fixture->GetUserData());
        mixedArray.push_back(particleBodyContact->normal.x);
        mixedArray.push_back(particleBodyContact->normal.y);
    }
    virtual void BeginContact(b2ParticleSystem* particleSystem,
                              b2ParticleContact* particleContact) {
        ++particleContacts;
        particleArray.push_back(particleSystem->MyIndex);
        particleArray.push_back(particleContact->GetIndexA());
        particleArray.push_back(particleContact->GetIndexB());
        //particleArray.push_back(particleContact->GetNormal().x);
        //particleArray.push_back(particleContact->GetNormal().y);
    }
    
    float* GetData() {
        
        lengthsArray.push_back(fixtureContacts);
        lengthsArray.push_back(mixedContacts);
        lengthsArray.push_back(particleContacts);
        
        returnArray.reserve(lengthsArray.size() + fixturesArray.size() + mixedArray.size() + particleArray.size());
        returnArray.insert(returnArray.end(), lengthsArray.begin(), lengthsArray.end());
        returnArray.insert(returnArray.end(), fixturesArray.begin(), fixturesArray.end());
        returnArray.insert(returnArray.end(), mixedArray.begin(), mixedArray.end());
        returnArray.insert(returnArray.end(), particleArray.begin(), particleArray.end());
        
        lengthsArray.clear();
        fixturesArray.clear();
        mixedArray.clear();
        particleArray.clear();
        
        fixtureContacts = 0;
        mixedContacts = 0;
        particleContacts = 0;
        
        infoArray = &returnArray[0];
        returnArray.clear();
        return infoArray;
    }
private:
    int fixtureContacts;
    int mixedContacts;
    int particleContacts;
    float* infoArray;
    std::vector<float> fixturesArray;
    std::vector<float> mixedArray;
    std::vector<float> particleArray;
    std::vector<float> lengthsArray;
    std::vector<float> returnArray;
};



#pragma region GlobalVariables
b2NewRaycastCallback* newRC;
float* positionArray;
int* returnArray;
#pragma endregion

#pragma region World

extern "C"  void* CreateWorld(float gravX, float gravY) {
    b2Vec2 gravity(gravX, gravY);
    b2World* world = new b2World(gravity);
    return static_cast<void*>(world);
}

extern "C"  void Step(void* worldPointer, float32 timeStep, int32 velocityIterations, int32 positionIterations) {
    b2World* world = static_cast<b2World*>(worldPointer);
    world->Step(timeStep, velocityIterations, positionIterations);
}

extern "C"  void StepWithParticleIterations(void* worldPointer, float32 timeStep, int32 velocityIterations, int32 positionIterations, int32 particleIterations) {
    b2World* world = static_cast<b2World*>(worldPointer);
    world->Step(timeStep, velocityIterations, positionIterations, particleIterations);
}

extern "C"  void* SetContactListener(void* worldPointer) {
    b2World* world = static_cast<b2World*>(worldPointer);
    b2NewContactListener* cL = new b2NewContactListener();
    world->SetContactListener(cL);
    return static_cast<void*>(cL);
}

extern "C"  float* UpdateContactListener(void* contactPointer) {
    b2NewContactListener* cL = static_cast<b2NewContactListener*>(contactPointer);
    return cL->GetData();
}

extern "C"  bool GetAllowSleeping(void* worldPointer) {
    b2World* world = static_cast<b2World*>(worldPointer);
    return world->GetAllowSleeping();
}
extern "C"  void SetAllowSleeping(void* worldPointer, bool flag) {
    b2World* world = static_cast<b2World*>(worldPointer);
    world->SetAllowSleeping(flag);
}

extern "C"  float* GetWorldGravity(void* worldPointer) {
    b2World* world = static_cast<b2World*>(worldPointer);
    float* returnArray = new float[2];
    returnArray[0] = world->GetGravity().x;
    returnArray[1] = world->GetGravity().y;
    return returnArray;
}
extern "C"  void SetWorldGravity(void* worldPointer, float x, float y) {
    b2World* world = static_cast<b2World*>(worldPointer);
    world->SetGravity(b2Vec2(x, y));
}


extern "C"  int End(void* worldPointer) {
    b2World* world = static_cast<b2World*>(worldPointer);
    if (world)
        delete world;
    return 0;
}

#pragma endregion

#pragma region ParticleSystems

extern "C"  int32 GetParticleIterations(float32 gravity, float32 particleRadius, float32 timeStep) {
    return b2CalculateParticleIterations(gravity, particleRadius, timeStep);
}

extern "C"  float* GetParticlePositions(void* particlesPointer) {
    
    if (positionArray != NULL)
    {
        delete positionArray;
    }
    b2ParticleSystem* particles = static_cast<b2ParticleSystem*>(particlesPointer);
    
    int numberOfParticles = particles->GetParticleCount();
    positionArray = new float[(numberOfParticles * 2) + 1];
    positionArray[0] = (float)numberOfParticles;
    b2Vec2* particlePositionBuffer = particles->GetPositionBuffer();
    
    int j = 1;
    for (int i = 0; i < numberOfParticles; ++i)
    {
        positionArray[j] = particlePositionBuffer[i].x;
        positionArray[j + 1] = particlePositionBuffer[i].y;
        j += 2;
    }
    
    return positionArray;
}


extern "C"  void* CreateParticleSystem(void* worldPointer, float radius, float damping, float gravityScale, int userData)
{
    b2World* world = static_cast<b2World*>(worldPointer);
    const b2ParticleSystemDef particleSystemDef;
    b2ParticleSystem* particleSystem = world->CreateParticleSystem(&particleSystemDef);
    particleSystem->SetRadius(radius);
    particleSystem->SetDamping(damping);
    particleSystem->SetGravityScale(gravityScale);
    particleSystem->SetIndex(userData);
    
    particleSystem->MyIndex = userData;
    //	particleSystem->SetUserData((void*)userData); HACK
    return static_cast<void*>(particleSystem);
}

extern "C"  void* CreateParticleSystem2(void* worldPointer, float radius, float damping, float gravityScale, int userData,
                                        float tennorm,float tenpres, float viscstr)
{
    b2World* world = static_cast<b2World*>(worldPointer);
    b2ParticleSystemDef particleSystemDef;
    particleSystemDef.surfaceTensionNormalStrength = tennorm;
    particleSystemDef.surfaceTensionPressureStrength = tenpres;
    particleSystemDef.viscousStrength = viscstr;
    b2ParticleSystem* particleSystem = world->CreateParticleSystem(&particleSystemDef);
    particleSystem->SetRadius(radius);
    particleSystem->SetDamping(damping);
    particleSystem->SetGravityScale(gravityScale);
    particleSystem->SetIndex(userData);
    
    particleSystem->MyIndex = userData;
    //	particleSystem->SetUserData((void*)userData); HACK
    return static_cast<void*>(particleSystem);
}

extern "C"  void* CreateParticleSystem3(void* worldPointer, float radius, float damping, float gravityScale, int userData,
                                        float presstr, float viscstr,float tenpres,float tennorm,float repulstr,float powstr,float ejectstr, float statpresstr,float statpresrelax, float colmixstr)
{
    b2World* world = static_cast<b2World*>(worldPointer);
    
    b2ParticleSystemDef particleSystemDef;
    
    particleSystemDef.pressureStrength = presstr;
    particleSystemDef.viscousStrength = viscstr;
    particleSystemDef.surfaceTensionPressureStrength = tenpres;
    particleSystemDef.surfaceTensionNormalStrength = tennorm;
    particleSystemDef.repulsiveStrength = repulstr;
    particleSystemDef.powderStrength = powstr;
    particleSystemDef.ejectionStrength = ejectstr;
    particleSystemDef.staticPressureStrength = statpresstr;
    particleSystemDef.staticPressureRelaxation = statpresrelax;
    particleSystemDef.colorMixingStrength = colmixstr;
    
    particleSystemDef.viscousStrength = viscstr;
    
    b2ParticleSystem* particleSystem = world->CreateParticleSystem(&particleSystemDef);
    particleSystem->SetRadius(radius);
    particleSystem->SetDamping(damping);
    particleSystem->SetGravityScale(gravityScale);
    particleSystem->SetIndex(userData);
    
    particleSystem->MyIndex = userData;
    //	particleSystem->SetUserData((void*)userData); HACK
    return static_cast<void*>(particleSystem);
}

extern "C"  void SetStaticPressureIterations(void* systemPointer,int iterations)
{
    b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(systemPointer);
    parts->SetStaticPressureIterations(iterations);
}

extern "C"  void SetAllParticleLifetimes(void* systemPointer, float time) {
    b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(systemPointer);
    int numberOfParticles = parts->GetParticleCount();
    for (int i = 0; i < numberOfParticles; ++i) {
        parts->SetParticleLifetime(i, time);
    }
    parts->SetDestructionByAge(true);
}

extern "C"  void SetDestructionByAge(void* systemPointer, bool toggle) {
    b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(systemPointer);
    parts->SetDestructionByAge(toggle);
}

extern "C"  bool GetDestructionByAge(void* systemPointer) {
    b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(systemPointer);
    return parts->GetDestructionByAge();
}

extern "C"  int DestroyParticlesInShape(void* systemPointer, void* shapePointer, float x, float y, float rot, bool call) {
    b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(systemPointer);
    b2Shape* shape = static_cast<b2Shape*>(shapePointer);
    b2Vec2 position = b2Vec2(x, y);
    b2Rot rotation = b2Rot(rot);
    b2Transform transform = b2Transform(position, rotation);
    return parts->DestroyParticlesInShape(*shape, transform, call);
}

extern "C"  void SetParticleSystemIndex(void* partsysPointer, int userData) {
    b2ParticleSystem* sys = static_cast<b2ParticleSystem*>(partsysPointer);
    
    sys->MyIndex = userData;
}

extern "C"  int* GetParticlesInShape(void* systemPointer, void* shapePointer, float x, float y, float rot)
{
    if (returnArray != NULL)
    {
        delete returnArray;
    }
    b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(systemPointer);
    b2Shape* shape = static_cast<b2Shape*>(shapePointer);
    b2Vec2 position = b2Vec2(x, y);
    b2Rot rotation = b2Rot(rot);
    b2Transform transform = b2Transform(position, rotation);
    
    const b2Vec2* positions = parts->GetPositionBuffer();
    int numparts = parts->GetParticleCount();
    
    std::vector<int> infoVector;
    int count = 0;
    
    for (int i = 0; i < numparts; i++)
    {
        if (shape->TestPoint(transform, positions[i]))
        {
            infoVector.push_back(i);
            count++;
        }
    }
    
    returnArray = new int[count + 1];
    returnArray[0] = count;
    for (int i = 0; i < count; i++)
    {
        returnArray[i + 1] = infoVector[i];
    }
    
    return returnArray;
}

extern "C"  float* GetParticlePositionsAndColors(void* particlesPointer) {
    
    if (positionArray != NULL)
    {
        delete positionArray;
    }
    
    b2ParticleSystem* particles = static_cast<b2ParticleSystem*>(particlesPointer);
    
    int numberOfParticles = particles->GetParticleCount();
    positionArray = new float[(numberOfParticles * 6) + 1];
    positionArray[0] = (float)numberOfParticles;
    b2Vec2* particlePositionBuffer = particles->GetPositionBuffer();
    b2ParticleColor* particleColorBuffer = particles->GetColorBuffer();
    
    int j = 1;
    for (int i = 0; i < numberOfParticles; ++i)
    {
        positionArray[j] = particlePositionBuffer[i].x;
        positionArray[j + 1] = particlePositionBuffer[i].y;
        positionArray[j + 2] = particleColorBuffer[i].r;
        positionArray[j + 3] = particleColorBuffer[i].g;
        positionArray[j + 4] = particleColorBuffer[i].b;
        positionArray[j + 5] = particleColorBuffer[i].a;
        j += 6;
    }
    return positionArray;
}

extern "C"  float* GetParticlesDetails(void* particlesPointer,bool position,bool color,bool age,bool weight,bool velocity,bool userdata)
{
    if (positionArray != NULL)
    {
        delete positionArray;
    }
    b2ParticleSystem* particles = static_cast<b2ParticleSystem*>(particlesPointer);
    
    int datanum  = 0;
    int numberOfParticles = particles->GetParticleCount();
    
    if (position) datanum +=2;
    if (color) datanum +=4;
    if (age) datanum +=1;
    if (weight) datanum +=1;
    if (velocity) datanum +=2;
    if (userdata) datanum +=1;
    
    positionArray = new float[(numberOfParticles * datanum) + 1];
    positionArray[0] = (float)numberOfParticles;
    int curpos = 1;
    
    if (position)
    {
        b2Vec2* particlePositionBuffer = particles->GetPositionBuffer();
        for (int i = 0; i < numberOfParticles; i++)
        {
            positionArray[curpos +(i*2)] = particlePositionBuffer[i].x;
            positionArray[curpos +(i*2)+1] = particlePositionBuffer[i].y;
        }
        curpos += numberOfParticles*2;
    }
    if (color)
    {
        b2ParticleColor* particleColorBuffer = particles->GetColorBuffer();
        for (int i = 0; i < numberOfParticles; i++)
        {
            positionArray[curpos +(i*4)] = particleColorBuffer[i].r;
            positionArray[curpos +(i*4)+1] = particleColorBuffer[i].g;
            positionArray[curpos +(i*4)+2] = particleColorBuffer[i].b;
            positionArray[curpos +(i*4)+3] = particleColorBuffer[i].a;
        }
        curpos += numberOfParticles*4;
    }
    if (age)
    {
        for (int i = 0; i < numberOfParticles; i++)
        {
            positionArray[curpos +i] = particles->GetParticleLifetime(i);
        }
        curpos += numberOfParticles;
    }
    if (weight)
    {
        float* particleweightBuffer = particles->GetWeightBuffer();
        for (int i = 0; i < numberOfParticles; i++)
        {
            positionArray[curpos +i] = particleweightBuffer[i];
            
        }
        curpos += numberOfParticles;
    }
    if (velocity)
    {
        b2Vec2* particlevelBuffer = particles->GetVelocityBuffer();
        for (int i = 0; i < numberOfParticles; i++)
        {
            positionArray[curpos +(i*2)] = particlevelBuffer[i].x;
            positionArray[curpos +(i*2)+1] = particlevelBuffer[i].y;
        }
        curpos += numberOfParticles*2;
    }
    if (userdata)
    {
        void** userdata = particles->GetUserDataBuffer();
        for (int i = 0; i < numberOfParticles; i++)
        {
            int num = (int64)userdata[i];
            positionArray[curpos +i] = (float)num;
        }
        curpos += numberOfParticles;
    }
    
    return positionArray;
}


extern "C"  void DeleteParticleSystem(void* worldPointer, void* particlesPointer) {
    b2World* world = static_cast<b2World*>(worldPointer);
    b2ParticleSystem* particles = static_cast<b2ParticleSystem*>(particlesPointer);
    world->DestroyParticleSystem(particles);
}

extern "C"  int GetNumberOfParticles(void* particlesPointer) {
    b2ParticleSystem* particles = static_cast<b2ParticleSystem*>(particlesPointer);
    
    return particles->GetParticleCount();
}

extern "C"  void SetAllParticleFlags(void* particlesPointer, int flags) {
    b2ParticleSystem* particles = static_cast<b2ParticleSystem*>(particlesPointer);
    int numParts = particles->GetParticleCount();
    for (int i = 0; i < numParts; ++i) {
        particles->SetParticleFlags(i, flags);
    }
    //particles->LiquidFunUpdatePairsAndTriads(particles->GetParticleCount() - 1); HACK
}

extern "C"  int* GetParticleSystemContacts(void* particlesPointer) {
    
    if (returnArray != NULL)
    {
        delete returnArray;
    }
    
    b2ParticleSystem* particles = static_cast<b2ParticleSystem*>(particlesPointer);
    int numContacts = particles->GetContactCount();
    const b2ParticleContact* contactList = particles->GetContacts();
    void** userdata = particles->GetUserDataBuffer();
    
    returnArray = new int[(numContacts * 4) + 1];
    returnArray[0] = numContacts;
    for (int i = 0; i < numContacts; ++i)
    {
        returnArray[(i * 4) + 1] = contactList[i].GetIndexA();
        returnArray[(i * 4) + 2] = contactList[i].GetIndexB();
        returnArray[(i * 4) + 3] = (int64)(userdata[contactList[i].GetIndexA()]);
        returnArray[(i * 4) + 4] = (int64)(userdata[contactList[i].GetIndexB()]);
    }
    return returnArray;
}

extern "C"  float* GetParticleSystemBodyContacts(void* particlesPointer) {
    
    if (positionArray != NULL)
    {
        delete positionArray;
    }
    
    b2ParticleSystem* particles = static_cast<b2ParticleSystem*>(particlesPointer);
    
    int count = particles->GetBodyContactCount();
    const b2ParticleBodyContact* contactList = particles->GetBodyContacts();
    void** userdata = particles->GetUserDataBuffer();
    
    positionArray = new float[(count*7) +1];
    positionArray[0] = (float)count;
    
    for (int i = 0; i < count; ++i)
    {
        positionArray[(i * 7) + 1] = (float)contactList[i].index;
        positionArray[(i * 7) + 2] = (float)((int64)(userdata[contactList[i].index]));
        positionArray[(i * 7) + 3] = (float)((int64)contactList[i].body->GetUserData());
        positionArray[(i * 7) + 4] = (float)((int64)contactList[i].fixture->GetUserData());
        positionArray[(i * 7) + 5] =  contactList[i].normal.x;
        positionArray[(i * 7) + 6] =  contactList[i].normal.y;
        positionArray[(i * 7) + 7] =  contactList[i].weight;
    }
    
    return positionArray;
}

extern "C"  int GetStuckCandidateCount(void* particlesPointer) {
    b2ParticleSystem* particles = static_cast<b2ParticleSystem*>(particlesPointer);
    return particles->GetStuckCandidateCount();
}

extern "C"  void SetStuckThreshold(void* particlesPointer, int iterations) {
    b2ParticleSystem* particles = static_cast<b2ParticleSystem*>(particlesPointer);
    particles->SetStuckThreshold(iterations);
}

extern "C"  void SetMaxParticleCount(void* particlesPointer, int count) {
    b2ParticleSystem* system = static_cast<b2ParticleSystem*>(particlesPointer);
    system->SetMaxParticleCount(count);
}
extern "C"  int GetMaxParticleCount(void* particlesPointer) {
    b2ParticleSystem* system = static_cast<b2ParticleSystem*>(particlesPointer);
    return system->GetMaxParticleCount();
}

#pragma endregion

#pragma region OneOrMoreParticles

extern "C"  void CreateParticleInSystem(void* systemPointer, int flags, float posX, float posY, float velX, float velY, int r, int g, int b, int a, float lifetime) {
    b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(systemPointer);
    b2Vec2 pos = b2Vec2(posX, posY);
    b2Vec2 vel = b2Vec2(velX, velY);
    b2ParticleDef pd;
    pd.flags = static_cast<b2ParticleFlag>(flags);
    pd.position = pos;
    pd.velocity = vel;
    pd.lifetime = lifetime;
    pd.color.Set((uint8)r, (uint8)g, (uint8)b, (uint8)a);
    parts->CreateParticle(pd);
}

extern "C"  void DestroySelectedParticles(void* particlesPointer, int* indexArray) {
    b2ParticleSystem* particles = static_cast<b2ParticleSystem*>(particlesPointer);
    for (int i = 1; i < indexArray[0]+1; ++i) {
        particles->DestroyParticle(indexArray[i]);
    }
}
extern "C"  void ApplyForceToSelectedParticles(void* particlesPointer, int* indexArray, float x, float y) {
    b2ParticleSystem* particles = static_cast<b2ParticleSystem*>(particlesPointer);
    b2Vec2 force = b2Vec2(x, y);
    for (int i = 1; i < indexArray[0]+1; ++i) {
        particles->ParticleApplyForce(indexArray[i], force);
    }
}
extern "C"  void ApplyLinearImpulseToSelectedParticles(void* particlesPointer, int* indexArray, float x, float y) {
    b2ParticleSystem* particles = static_cast<b2ParticleSystem*>(particlesPointer);
    b2Vec2 force = b2Vec2(x, y);
    for (int i = 1; i < indexArray[0]+1; ++i) {
        particles->ParticleApplyLinearImpulse(indexArray[i], force);
    }
}

extern "C"  void SetSelectedParticleFlags(void* particlesPointer, int* indexArray, int flags) {
    b2ParticleSystem* particles = static_cast<b2ParticleSystem*>(particlesPointer);
    for (int i = 1; i < indexArray[0]+1; ++i) {
        particles->SetParticleFlags(indexArray[i], flags);
    }
    //particles->LiquidFunUpdatePairsAndTriads(particles->GetParticleCount() - 1); HACK
}

extern "C"  void SetSelectedParticleColor(void* particlesPointer, int* indexArray, int r, int g, int b, int a) {
    b2ParticleSystem* particles = static_cast<b2ParticleSystem*>(particlesPointer);
    b2ParticleColor* colours = particles->GetColorBuffer();
    
    for (int i = 1; i < indexArray[0]+1; ++i)
    {
        colours[indexArray[i]].r = r;
        colours[indexArray[i]].g = g;
        colours[indexArray[i]].b = b;
        colours[indexArray[i]].a = a;
    }
}

extern "C"  void SetSelectedParticleUserData(void* particlesPointer, int* indexArray, int ud) {
    b2ParticleSystem* particles = static_cast<b2ParticleSystem*>(particlesPointer);
    void** datas = particles->GetUserDataBuffer();
    
    for (int i = 1; i < indexArray[0]+1; ++i)
    {
        datas[indexArray[i]] = (void*)ud;
    }
}

extern "C"  void ExplodeSelectedParticles(void* particlesPointer, int* indexArray, float centreX, float centreY, float strenght) {
    b2ParticleSystem* particles = static_cast<b2ParticleSystem*>(particlesPointer);
    b2Vec2* ppb = particles->GetPositionBuffer();
    for (int i = 1; i < indexArray[0]+1; ++i)
    {
        int ind = indexArray[i];
        b2Vec2 force = b2Vec2(ppb[ind].x - centreX, ppb[ind].y - centreY);
        force.Normalize();
        force *= strenght;
        particles->ParticleApplyForce(ind, force);
    }
}

extern "C"  float* GetSelectedParticlesDetails(void* particlesPointer, int* indexArray ,bool position,bool color,bool age,bool weight,bool velocity,bool userdata)
{
    if (positionArray != NULL)
    {
        delete positionArray;
    }
    b2ParticleSystem* particles = static_cast<b2ParticleSystem*>(particlesPointer);
    
    int datanum  = 0;
    int numberOfParticles = indexArray[0];
    
    if (position) datanum +=2;
    if (color) datanum +=4;
    if (age) datanum +=1;
    if (weight) datanum +=1;
    if (velocity) datanum +=2;
    if (userdata) datanum +=1;
    
    positionArray = new float[(numberOfParticles * datanum) + 1];
    positionArray[0] = (float)numberOfParticles;
    int curpos = 1;
    
    if (position)
    {
        b2Vec2* particlePositionBuffer = particles->GetPositionBuffer();
        for (int i = 0; i < numberOfParticles; i++)
        {
            positionArray[curpos +(i*2)] = particlePositionBuffer[indexArray[i+1]].x;
            positionArray[curpos +(i*2)+1] = particlePositionBuffer[indexArray[i+1]].y;
        }
        curpos += numberOfParticles*2;
    }
    if (color)
    {
        b2ParticleColor* particleColorBuffer = particles->GetColorBuffer();
        for (int i = 0; i < numberOfParticles; i++)
        {
            positionArray[curpos +(i*4)] = particleColorBuffer[indexArray[i+1]].r;
            positionArray[curpos +(i*4)+1] = particleColorBuffer[indexArray[i+1]].g;
            positionArray[curpos +(i*4)+2] = particleColorBuffer[indexArray[i+1]].b;
            positionArray[curpos +(i*4)+3] = particleColorBuffer[indexArray[i+1]].a;
        }
        curpos += numberOfParticles*4;
    }
    if (age)
    {
        for (int i = 0; i < numberOfParticles; i++)
        {
            positionArray[curpos +i] = particles->GetParticleLifetime(indexArray[i+1]);
        }
        curpos += numberOfParticles;
    }
    if (weight)
    {
        float* particleweightBuffer = particles->GetWeightBuffer();
        for (int i = 0; i < numberOfParticles; i++)
        {
            positionArray[curpos +i] = particleweightBuffer[indexArray[i+1]];
            
        }
        curpos += numberOfParticles;
    }
    if (velocity)
    {
        b2Vec2* particlevelBuffer = particles->GetVelocityBuffer();
        for (int i = 0; i < numberOfParticles; i++)
        {
            positionArray[curpos +(i*2)] = particlevelBuffer[indexArray[i+1]].x;
            positionArray[curpos +(i*2)+1] = particlevelBuffer[indexArray[i+1]].y;
        }
        curpos += numberOfParticles*2;
    }
    if (userdata)
    {
        void** userdata = particles->GetUserDataBuffer();
        for (int i = 0; i < numberOfParticles; i++)
        {
            int num = (int64)userdata[indexArray[i+1]];
            positionArray[curpos +i] = (float)num;
        }
        curpos += numberOfParticles;
    }
    
    return positionArray;
}

#pragma endregion

#pragma region ParticleGroups

extern "C"  void* CreateParticleGroup(void* particlesPointer, int particleTypes, int groupTypes, float angle, float strength, float angVel, float linVelX, float linVelY, void* shape, int r, int g, int b, int a, float stride, float lifetime, int userData) {
    b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(particlesPointer);
    b2Shape* m_shape = static_cast<b2Shape*>(shape);
    b2ParticleGroupFlag bgf;
    bgf = static_cast<b2ParticleGroupFlag>(groupTypes);
    b2ParticleFlag bf;
    bf = static_cast<b2ParticleFlag>(particleTypes);
    b2ParticleGroupDef pd;
    pd.flags = bf;
    pd.groupFlags = bgf;
    pd.shape = m_shape;
    pd.angle = angle;
    pd.strength = strength;
    pd.angularVelocity = angVel;
    pd.linearVelocity = b2Vec2(linVelX, linVelY);
    pd.stride = stride;
    pd.lifetime = lifetime;
    pd.color.Set((uint8)r, (uint8)g, (uint8)b, (uint8)a);
    pd.userData = (void*)userData;
    b2ParticleGroup* particleGroup = parts->CreateParticleGroup(pd);
    return static_cast<void*>(particleGroup);
}

extern "C"  void JoinParticleGroups(void* particlesPointer, void* groupAPointer, void* groupBPointer)
{
    b2ParticleGroup* groupA = static_cast<b2ParticleGroup*>(groupAPointer);
    b2ParticleGroup* groupB = static_cast<b2ParticleGroup*>(groupBPointer);
    
    b2ParticleSystem* system = static_cast<b2ParticleSystem*>(particlesPointer);
    system->JoinParticleGroups(groupA, groupB);
}

extern "C"  int* AreParticlesInGroup(void* groupPointer, int* indices) {
    b2ParticleGroup* group = static_cast<b2ParticleGroup*>(groupPointer);
    std::vector<int> flags;
    for (int i = 1; i < indices[0]+1; ++i) {
        flags.push_back(group->ContainsParticle(indices[i]) ? 1 : 0);
    }
    int* flagArray = &flags[0];
    return flagArray;
}

extern "C"  void SetParticleFlagsInGroup(void* particlesPointer, void* groupPointer, int flags) {
    b2ParticleGroup* group = static_cast<b2ParticleGroup*>(groupPointer);
    b2ParticleSystem* system = static_cast<b2ParticleSystem*>(particlesPointer);
    int numParts = system->GetParticleCount();
    for (int i = 0; i < numParts; ++i) {
        if (group->ContainsParticle(i)) {
            system->SetParticleFlags(i, flags);
        }
    }
    //system->LiquidFunUpdatePairsAndTriads(system->GetParticleCount() - 1); HACK
}

extern "C"  void SetParticleLifetimesInGroup(void* particlesPointer, void* groupPointer, int lifetime) {
    b2ParticleGroup* group = static_cast<b2ParticleGroup*>(groupPointer);
    b2ParticleSystem* system = static_cast<b2ParticleSystem*>(particlesPointer);
    int numParts = system->GetParticleCount();
    for (int i = 0; i < numParts; ++i) {
        if (group->ContainsParticle(i)) {
            system->SetParticleLifetime(i, lifetime);
        }
    }
}

extern "C"  int GetparticleGroupCount(void* groupPointer)
{
    b2ParticleGroup* group = static_cast<b2ParticleGroup*>(groupPointer);
    return group->GetParticleCount();
}

extern "C"  void ApplyForceToParticleGroup(void* groupPointer, float forceX, float forceY) {
    b2ParticleGroup* group = static_cast<b2ParticleGroup*>(groupPointer);
    b2Vec2 force = b2Vec2(forceX, forceY);
    group->ApplyForce(force);
}
extern "C"  void ApplyLinearImpulseToParticleGroup(void* groupPointer, float forceX, float forceY) {
    b2ParticleGroup* group = static_cast<b2ParticleGroup*>(groupPointer);
    b2Vec2 impulse = b2Vec2(forceX, forceY);
    group->ApplyLinearImpulse(impulse);
}

extern "C"  float* GetParticleGroupPosition(void* particleGroupPointer)
{
    
    if (positionArray != NULL)
    {
        delete positionArray;
    }
    
    b2ParticleGroup* pGroup = static_cast<b2ParticleGroup*>(particleGroupPointer);
    b2Vec2 vec = pGroup->GetPosition();
    positionArray = new float[2];
    positionArray[0] = vec.x;
    positionArray[1] = vec.y;
    
    return positionArray;
}

extern "C"  float* GetParticleGroupVelocity(void* particleGroupPointer)
{
    if (positionArray != NULL)
    {
        delete positionArray;
    }
    
    b2ParticleGroup* pGroup = static_cast<b2ParticleGroup*>(particleGroupPointer);
    b2Vec2 vec = pGroup->GetLinearVelocity() ;
    positionArray = new float[2];
    positionArray[0] = vec.x;
    positionArray[1] = vec.y;
    
    return positionArray;
}

extern "C"  void DeleteParticlesInGroup(void* particleGroupPointer) {
    b2ParticleGroup* pGroup = static_cast<b2ParticleGroup*>(particleGroupPointer);
    pGroup->DestroyParticles();
}

#pragma endregion

#pragma region Particles Superceded




extern "C"  void SetSingleParticleLifetime(void* systemPointer, int index, float time) {
    b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(systemPointer);
    parts->SetParticleLifetime(index, time);
}


/*
 extern "C"  int* GetParticlesInShape(void* systemPointer, void* shapePointer, float x, float y, float rot) {
 b2ParticleSystem* parts = static_cast<b2ParticleSystem*>(systemPointer);
 b2Shape* shape = static_cast<b2Shape*>(shapePointer);
 b2Vec2 position = b2Vec2(x, y);
 b2Rot rotation = b2Rot(rot);
 b2Transform transform = b2Transform(position, rotation);
 std::vector<int> ints = parts->GetParticlesInShape(*shape, transform);
 int* arr = &ints[0];
 int* arr1 = new int[1];
 arr1[0] = 42;
 return arr1;
 }
 */



extern "C"  void SetParticleFlagsUpToLimit(void* particlesPointer, int flags, int limit) {
    b2ParticleSystem* particles = static_cast<b2ParticleSystem*>(particlesPointer);
    for (int i = 0; i < limit; ++i) {
        particles->SetParticleFlags(i, flags);
    }
    //particles->LiquidFunUpdatePairsAndTriads(particles->GetParticleCount() - 1); HACK
}


/*
 extern "C"  float* GetSelectedParticleDetails(void* particlesPointer, float* indexArray) {
 b2ParticleSystem* system = static_cast<b2ParticleSystem*>(particlesPointer);
 std::vector<float> indexList;
 std::vector<float> positions;
 for (int i = 1; i < indexArray[0]; ++i) {
 indexList.push_back(indexArray[i]);
 }
 
 for (int i = 0; i < indexList.size(); ++i) {
 positions.push_back(system->GetParticlePositionX(indexList[i]));
 positions.push_back(system->GetParticlePositionY(indexList[i]));
 }
 float* returnArray = &positions[0];
 return returnArray;
 }
 
 
 
 
 extern "C"  float* GetSpecificParticlePositions(void* particlesPointer, float* indexArray) {
	b2ParticleSystem* system = static_cast<b2ParticleSystem*>(particlesPointer);
	std::vector<float> indexList;
	std::vector<float> positions;
	for (int i = 1; i < indexArray[0]; ++i) {
 indexList.push_back(indexArray[i]);
	}
 
	for (int i = 0; i < indexList.size(); ++i) {
 positions.push_back(system->GetParticlePositionX(indexList[i]));
 positions.push_back(system->GetParticlePositionY(indexList[i]));
	}
	float* returnArray = &positions[0];
	return returnArray;
 }*/

extern "C"  void ApplyLinearImpulseToParticles(void* particlesPointer, int first, int last, float impulseX, float impulseY) {
    b2ParticleSystem* system = static_cast<b2ParticleSystem*>(particlesPointer);
    b2Vec2 impulse = b2Vec2(impulseX, impulseY);
    system->ApplyLinearImpulse(first, last, impulse);
}
extern "C"  void ApplyForceToParticles(void* particlesPointer, int first, int last, float impulseX, float impulseY) {
    b2ParticleSystem* system = static_cast<b2ParticleSystem*>(particlesPointer);
    b2Vec2 impulse = b2Vec2(impulseX, impulseY);
    system->ApplyForce(first, last, impulse);
}
extern "C"  void ApplyForceToParticle(void* particlesPointer, int index, float impulseX, float impulseY) {
    b2ParticleSystem* system = static_cast<b2ParticleSystem*>(particlesPointer);
    b2Vec2 impulse = b2Vec2(impulseX, impulseY);
    system->ParticleApplyForce(index, impulse);
}
extern "C"  void ApplyLinearImpulseToParticle(void* particlesPointer, int index, float impulseX, float impulseY) {
    b2ParticleSystem* system = static_cast<b2ParticleSystem*>(particlesPointer);
    b2Vec2 impulse = b2Vec2(impulseX, impulseY);
    system->ParticleApplyLinearImpulse(index, impulse);
}

#pragma endregion

#pragma region Shapes

extern "C"  void* GetBoxShapeDef(float width, float height, float centreX, float centreY, float angle) {
    b2Vec2 centre = b2Vec2(centreX, centreY);
    b2PolygonShape* shape = new b2PolygonShape();
    shape->SetAsBox(width, height, centre, angle);
    return static_cast<void*>(shape);
}

extern "C"  void* GetCircleShapeDef(float radius, float centreX, float centreY) {
    b2Vec2 centre = b2Vec2(centreX, centreY);
    b2CircleShape* shape = new b2CircleShape();
    shape->m_p = centre;
    shape->m_radius = radius;
    return static_cast<void*>(shape);
}

extern "C"  void* GetChainShapeDef(float* vertArray, bool loop) {
    b2ChainShape* shape = new b2ChainShape();
    int numberOfVertices = vertArray[0];
    b2Vec2* vertices = new b2Vec2[numberOfVertices];
    
    int j = 0;
    for (int i = 1; i < (vertArray[0] * 2); i += 2)
    {
        vertices[j] = b2Vec2(vertArray[i], vertArray[i + 1]);
        ++j;
    }
    
    if (loop) {
        shape->CreateLoop(vertices, numberOfVertices);
    }
    else {
        shape->CreateChain(vertices, numberOfVertices);
    }
    return static_cast<void*>(shape);
}

extern "C"  void* GetPolygonShapeDef(float* vertArray) {
    b2PolygonShape* shape = new b2PolygonShape();
    int numberOfVertices = vertArray[0];
    b2Vec2* vertices = new b2Vec2[numberOfVertices];
    
    int j = 0;
    for (int i = 1; i < (vertArray[0] * 2); i += 2)
    {
        vertices[j] = b2Vec2(vertArray[i], vertArray[i + 1]);
        ++j;
    }
    
    shape->Set(vertices, numberOfVertices);
    return static_cast<void*>(shape);
}

extern "C"  void* GetEdgeShapeDef(float x1, float y1, float x2, float y2) {
    b2Vec2 vec1 = b2Vec2(x1, y1);
    b2Vec2 vec2 = b2Vec2(x2, y2);
    b2EdgeShape* shape = new b2EdgeShape();
    shape->Set(vec1, vec2);
    return static_cast<void*>(shape);
}

extern "C"  void* GetEllipseShapeDef(float outerRadius, float divisions) {
    /*b2ChainShape* chainShape;
     std::vector<b2Vec2> vertices;
     const float32 SPIKE_DEGREE = 2 * 3.14159265358979323846f / 180;
     for (int idx = 0; idx < divisions; idx++) {
     float32 angle = ((3.14159265358979323846f * 2) / divisions)*idx;
     float32 xPos, yPos;
     
     xPos = outerRadius*cosf(angle);
     yPos = outerRadius*sinf(angle);
     vertices.push_back(b2Vec2(xPos, yPos));
     }
     vertices.push_back(vertices[0]);
     chainShape->CreateChain(&vertices[0], vertices.size());
     return static_cast<void*>(chainShape);*/
    
    b2PolygonShape* shape = new b2PolygonShape();
    int numberOfVertices = divisions;
    b2Vec2* vertices = new b2Vec2[numberOfVertices];
    
    for (int idx = 0; idx < divisions; idx++) {
        float32 angle = ((3.14159265358979323846f * 2) / divisions)*idx;
        float32 xPos, yPos;
        
        xPos = outerRadius*cosf(angle);
        yPos = outerRadius*sinf(angle);
        vertices[idx] = b2Vec2(xPos, yPos);
    }
    
    shape->Set(vertices, numberOfVertices);
    return static_cast<void*>(shape);
    
}

extern "C"  float* GetPolyShapeCentroid(void* shapePointer) {
    float * positionArray = new float[2];
    b2PolygonShape* m_shape = static_cast<b2PolygonShape*>(shapePointer);
    positionArray[0] = m_shape->m_centroid.x;
    positionArray[1] = m_shape->m_centroid.y;
    return positionArray;
}

#pragma endregion

#pragma region Body

extern "C"  void* CreateBody(void* worldPointer, int type, float xPosition, float yPosition, float angle, float linearDamping, float angularDamping, bool allowSleep, bool fixedRotation, bool bullet, float gravityScale, int userData) {
    b2World* world = static_cast<b2World*>(worldPointer);
    b2Vec2 position = b2Vec2(xPosition, yPosition);
    b2BodyType bodyType;
    
    if (type == 1)
        bodyType = b2_dynamicBody;
    else if (type == 2)
        bodyType = b2_kinematicBody;
    else
        bodyType = b2_staticBody;
    
    b2BodyDef bd;
    bd.type = bodyType;
    bd.position = position;
    bd.angle = angle;
    bd.linearDamping = linearDamping;
    bd.angularDamping = angularDamping;
    bd.allowSleep = allowSleep;
    bd.fixedRotation = fixedRotation;
    bd.bullet = bullet;
    bd.gravityScale = gravityScale;
    bd.userData = (void*)userData;
    
    b2Body* m_body = world->CreateBody(&bd);
    return static_cast<void*>(m_body);
}

extern "C"  float* GetBodyInfo(void* bodyPointer) {
    
    if (positionArray != NULL)
    {
        delete positionArray;
    }
    
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    positionArray = new float[3];
    positionArray[0] = m_body->GetPosition().x;
    positionArray[1] = m_body->GetPosition().y;
    positionArray[2] = m_body->GetAngle();
    
    return positionArray;
}

extern "C"  float* GetAllBodyInfo(void** bodPointers, int numbodies)
{
    if (positionArray != NULL)
    {
        delete positionArray;
    }
    
    positionArray = new float[numbodies*3];
    
    for (int i = 0; i < numbodies; i++)
    {
        b2Body* m_body = static_cast<b2Body*>(bodPointers[i]);
        positionArray[i*3] = m_body->GetPosition().x;
        positionArray[(i*3)+1] = m_body->GetPosition().y;
        positionArray[(i*3)+2] = m_body->GetAngle();
    }
    
    return positionArray;
}

extern "C"  int* GetBodyContacts(void* bodyPointer)
{
    if (returnArray != NULL)
    {
        delete returnArray;
    }
    
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    //b2ContactEdge * contacts =  m_body->GetContactList() ;
    
    int numconts = 0;
    std::vector<int> conts;
    
    for ( b2ContactEdge * ce =  m_body->GetContactList(); ce; ce = ce->next)
    {
        numconts++;
        conts.push_back((int64)(ce->other->GetUserData()));
    }
    
    returnArray = new int[numconts+1];
    returnArray[0] = numconts;
    for (int i = 0; i < numconts; i++)
    {
        returnArray[i+1] = conts[i];
    }
    
    return returnArray;
    
}

extern "C"  int GetBodyContactsCount(void* bodyPointer)
{
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    //b2ContactEdge * contacts =  m_body->GetContactList() ;
    
    int numconts = 0;
    //std::vector<int> conts;
    //conts.push_back(numconts);
    
    for ( b2ContactEdge* ce =  m_body->GetContactList(); ce; ce = ce->next)
    {
        numconts++;
        //conts.push_back((int)ce->other->GetUserData());
    }
    //conts[0] = numconts;
    
    //int* returnArray = &conts[0];
    return numconts;
    
}

extern "C"  void ApplyForceToCentreOfBody(void* bodyPointer, float impulseX, float impulseY) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    b2Vec2 impulse = b2Vec2(impulseX, impulseY);
    m_body->ApplyForceToCenter(impulse, true);
}

extern "C"  void SetBodyAwake(void* bodyPointer, bool isAwake) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    m_body->SetAwake(isAwake);
}

extern "C"  void SetBodyType(void* bodyPointer, int type) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    
    b2BodyType bodyType;
    
    if (type == 1)
        bodyType = b2_dynamicBody;
    else if (type == 2)
        bodyType = b2_kinematicBody;
    else
        bodyType = b2_staticBody;
    
    m_body->SetType(bodyType);
}

extern "C"  bool GetBodyAwake(void* bodyPointer) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    return m_body->IsAwake();
}

extern "C"  void SetBodyActive(void* bodyPointer, bool isActive) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    m_body->SetActive(isActive);
}

extern "C"  bool GetBodyActive(void* bodyPointer) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    return m_body->IsActive();
}

extern "C"  void** GetBodyFixtures(void* bodyPointer) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    std::vector<void*> fixturesVec;
    fixturesVec.clear();
    for (b2Fixture* f = m_body->GetFixtureList(); f; f = f->GetNext()) {
        fixturesVec.push_back(static_cast<void*>(f));
    }
    void** fixturesArray = &fixturesVec[0];
    return fixturesArray;
}

extern "C"  int GetBodyFixturesCount(void* bodyPointer) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    int i = 0;
    for (b2Fixture* f = m_body->GetFixtureList(); f; f = f->GetNext()) {
        ++i;
    }
    return i;
}

extern "C"  int GetBodyUserData(void* bodyPointer) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    return (int64)m_body->GetUserData();
}

extern "C"  void SetBodyPosition(void* bodyPointer, float x, float y) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    b2Vec2 pos = b2Vec2(x, y);
    m_body->SetTransform(pos, m_body->GetAngle());
}

extern "C"  void SetBodyRotation(void* bodyPointer, float rotation) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    m_body->SetTransform(m_body->GetPosition(), rotation);
}

extern "C"  void SetBodyLinearVelocity(void* bodyPointer, float x, float y) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    b2Vec2 vel = b2Vec2(x, y);
    m_body->SetLinearVelocity(vel);
}
extern "C"  float* GetBodyLinearVelocity(void* bodyPointer) {
    
    if (positionArray != NULL)
    {
        delete positionArray;
    }
    
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    positionArray = new float[2];
    positionArray[0] = m_body->GetLinearVelocity().x;
    positionArray[1] = m_body->GetLinearVelocity().y;
    return positionArray;
}

extern "C"  void SetBodyLinearDamping(void* bodyPointer, float lD) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    m_body->SetLinearDamping(lD);
}
extern "C"  float GetBodyLinearDamping(void* bodyPointer) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    return m_body->GetLinearDamping();
}

extern "C"  void SetBodyAngularDamping(void* bodyPointer, float aD) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    m_body->SetAngularDamping(aD);
}
extern "C"  float GetBodyAngularDamping(void* bodyPointer) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    return m_body->GetAngularDamping();
}

extern "C"  void SetBodyAngularVelocity(void* bodyPointer, float w) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    m_body->SetAngularVelocity(w);
}
extern "C"  float GetBodyAngularVelocity(void* bodyPointer) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    return m_body->GetAngularVelocity();
}

extern "C"  void SetBodyGravityScale(void* bodyPointer, float scale) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    m_body->SetGravityScale(scale);
}
extern "C"  float GetBodyGravityScale(void* bodyPointer) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    return m_body->GetGravityScale();
}

extern "C"  void SetBodyIsBullet(void* bodyPointer, bool isBullet) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    m_body->SetBullet(isBullet);
}
extern "C"  bool GetBodyIsBullet(void* bodyPointer) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    return m_body->IsBullet();
}

extern "C"  void SetBodyFixedRotation(void* bodyPointer, bool flag) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    m_body->SetFixedRotation(flag);
}
extern "C"  bool GetBodyFixedRotation(void* bodyPointer) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    return m_body->IsFixedRotation();
}

extern "C"  void ApplyAngularImpulseToBody(void* bodyPointer, float impulse, bool wake) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    m_body->ApplyAngularImpulse(impulse, wake);
}
extern "C"  void ApplyForceToBody(void* bodyPointer, float forceX, float forceY, float posX, float posY, bool wake) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    b2Vec2 force = b2Vec2(forceX, forceY);
    b2Vec2 position = b2Vec2(posX, posY);
    m_body->ApplyForce(force, position, wake);
}
extern "C"  void ApplyLinearImpulseToBody(void* bodyPointer, float forceX, float forceY, float posX, float posY, bool wake) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    b2Vec2 force = b2Vec2(forceX, forceY);
    b2Vec2 position = b2Vec2(posX, posY);
    m_body->ApplyLinearImpulse(force, position, wake);
}
extern "C"  void ApplyTorqueToBody(void* bodyPointer, float torque, bool wake) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    m_body->ApplyTorque(torque, wake);
}

extern "C"  float GetBodyMass(void* bodyPointer) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    return m_body->GetMass();
}
extern "C"  float GetBodyInertia(void* bodyPointer) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    return m_body->GetInertia();
}

extern "C"  void SetBodyTransform(void* bodyPointer, float x, float y, float angle) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    b2Vec2 pos = b2Vec2(x, y);
    m_body->SetTransform(pos, angle);
}


extern "C"  void DeleteBody(void* worldPointer, void* bodyPointer) {
    b2World* world = static_cast<b2World*>(worldPointer);
    b2Body* body = static_cast<b2Body*>(bodyPointer);
    world->DestroyBody(body);
}

#pragma endregion

#pragma region Fixture

extern "C"  void* AddFixture(void* bodyPointer, int shapeType, void* shapePointer, float density, float friction, float restitution, bool isSensor, int userData) {
    b2Body* m_body = static_cast<b2Body*>(bodyPointer);
    b2FixtureDef fd;
    fd.density = density;
    fd.friction = friction;
    fd.isSensor = isSensor;
    fd.restitution = restitution;
    fd.userData = (void*)userData;
    if (shapeType == 0) {
        b2PolygonShape* aShape = static_cast<b2PolygonShape*>(shapePointer);
        b2PolygonShape shape = *aShape;
        fd.shape = &shape;
        b2Fixture* m_fixture = m_body->CreateFixture(&fd);
        return static_cast<void*>(m_fixture);
    }
    else if (shapeType == 1) {
        b2CircleShape* aShape = static_cast<b2CircleShape*>(shapePointer);
        b2CircleShape shape = *aShape;
        fd.shape = &shape;
        b2Fixture* m_fixture = m_body->CreateFixture(&fd);
        return static_cast<void*>(m_fixture);
    }
    else if (shapeType == 2) {
        b2EdgeShape* aShape = static_cast<b2EdgeShape*>(shapePointer);
        b2EdgeShape shape = *aShape;
        fd.shape = &shape;
        b2Fixture* m_fixture = m_body->CreateFixture(&fd);
        return static_cast<void*>(m_fixture);
    }
    else {
        b2ChainShape* aShape = static_cast<b2ChainShape*>(shapePointer);
        b2ChainShape shape = *aShape;
        fd.shape = &shape;
        b2Fixture* m_fixture = m_body->CreateFixture(&fd);
        return static_cast<void*>(m_fixture);
    }
    
}

extern "C"  float* GetFixtureInfo(void* fixturePointer) {
    
    if (positionArray != NULL)
    {
        delete positionArray;
    }
    
    b2Fixture* m_fixture = static_cast<b2Fixture*>(fixturePointer);
    positionArray = new float[3];
    positionArray[0] = m_fixture->GetBody()->GetPosition().x;
    positionArray[1] = m_fixture->GetBody()->GetPosition().y;
    positionArray[2] = m_fixture->GetBody()->GetAngle();
    
    return positionArray;
}

extern "C"  int GetFixtureUserData(void* fixturePointer) {
    b2Fixture* m_fixture = static_cast<b2Fixture*>(fixturePointer);
    return (int64)m_fixture->GetUserData();
}

extern "C"  bool TestPoint(void* fixturePointer, float x, float y) {
    b2Fixture* m_fixture = static_cast<b2Fixture*>(fixturePointer);
    b2Vec2 pos = b2Vec2(x, y);
    return m_fixture->TestPoint(pos);
}

extern "C"  void SetFixtureFilterData(void* fixturePointer, int16 groupIndex, uint16 categoryBits, uint16 maskBits) {
    b2Fixture* m_fixture = static_cast<b2Fixture*>(fixturePointer);
    b2Filter filter = b2Filter();
    filter.groupIndex = groupIndex;
    filter.maskBits = maskBits;
    filter.categoryBits = categoryBits;
    m_fixture->SetFilterData(filter);
}
extern "C"  uint16 GetFixtureGroupIndex(void* fixturePointer) {
    b2Fixture* m_fixture = static_cast<b2Fixture*>(fixturePointer);
    return m_fixture->GetFilterData().groupIndex;
}
extern "C"  uint16 GetFixtureMaskBits(void* fixturePointer) {
    b2Fixture* m_fixture = static_cast<b2Fixture*>(fixturePointer);
    return m_fixture->GetFilterData().maskBits;
}
extern "C"  uint16 GetFixtureCategoryBits(void* fixturePointer) {
    b2Fixture* m_fixture = static_cast<b2Fixture*>(fixturePointer);
    return m_fixture->GetFilterData().categoryBits;
}

extern "C"  bool GetFixtureIsSensor(void* fixturePointer) {
    b2Fixture* m_fixture = static_cast<b2Fixture*>(fixturePointer);
    return m_fixture->IsSensor();
}
extern "C"  void SetFixtureIsSensor(void* fixturePointer, bool flag) {
    b2Fixture* m_fixture = static_cast<b2Fixture*>(fixturePointer);
    m_fixture->SetSensor(flag);
}

extern "C"  float GetFixtureDensity(void* fixturePointer) {
    b2Fixture* m_fixture = static_cast<b2Fixture*>(fixturePointer);
    return m_fixture->GetDensity();
}
extern "C"  void SetFixtureDensity(void* fixturePointer, float density) {
    b2Fixture* m_fixture = static_cast<b2Fixture*>(fixturePointer);
    m_fixture->SetDensity(density);
}

extern "C"  float GetFixtureFriction(void* fixturePointer) {
    b2Fixture* m_fixture = static_cast<b2Fixture*>(fixturePointer);
    return m_fixture->GetFriction();
}
extern "C"  void SetFixtureFriction(void* fixturePointer, float friction) {
    b2Fixture* m_fixture = static_cast<b2Fixture*>(fixturePointer);
    m_fixture->SetFriction(friction);
}

extern "C"  float GetFixtureRestitution(void* fixturePointer) {
    b2Fixture* m_fixture = static_cast<b2Fixture*>(fixturePointer);
    return m_fixture->GetRestitution();
}
extern "C"  void SetFixtureRestitution(void* fixturePointer, float restitution) {
    b2Fixture* m_fixture = static_cast<b2Fixture*>(fixturePointer);
    m_fixture->SetRestitution(restitution);
}

extern "C"  void DeleteFixture(void* bodyPointer, void* fixturePointer) {
    b2Fixture* fixture = static_cast<b2Fixture*>(fixturePointer);
    b2Body* body = static_cast<b2Body*>(bodyPointer);
    body->DestroyFixture(fixture);
}
#pragma endregion

#pragma region Joints

#pragma region DistanceJoints
extern "C"  void* CreateDistanceJoint(void* worldPointer, void* bodyA, void* bodyB, float anchorAX, float anchorAY, float anchorBX, float anchorBY, float length, bool collideConnected) {
    b2World* world = static_cast<b2World*>(worldPointer);
    b2Body* m_bodyA = static_cast<b2Body*>(bodyA);
    b2Body* m_bodyB = static_cast<b2Body*>(bodyB);
    b2DistanceJoint* dj;
    b2DistanceJointDef jd;
    jd.bodyA = m_bodyA;
    jd.bodyB = m_bodyB;
    jd.localAnchorA.Set(anchorAX, anchorAY);
    jd.localAnchorB.Set(anchorBX, anchorBY);
    jd.collideConnected = collideConnected;
    jd.length = length;
    dj = (b2DistanceJoint*)world->CreateJoint(&jd);
    return static_cast<void*>(dj);
}
extern "C"  void SetDistanceJointFrequency(void* joint, float frequency) {
    b2DistanceJoint* m_joint = static_cast<b2DistanceJoint*>(joint);
    m_joint->SetFrequency(frequency);
}
extern "C"  float GetDistanceJointFrequency(void* joint) {
    b2DistanceJoint* m_joint = static_cast<b2DistanceJoint*>(joint);
    return m_joint->GetFrequency();
}
extern "C"  void SetDistanceJointDampingRatio(void* joint, float ratio) {
    b2DistanceJoint* m_joint = static_cast<b2DistanceJoint*>(joint);
    m_joint->SetDampingRatio(ratio);
}
extern "C"  float GetDistanceJointDampingRatio(void* joint) {
    b2DistanceJoint* m_joint = static_cast<b2DistanceJoint*>(joint);
    return m_joint->GetDampingRatio();
}
extern "C"  void SetDistanceJointLength(void* joint, float length) {
    b2DistanceJoint* m_joint = static_cast<b2DistanceJoint*>(joint);
    m_joint->SetLength(length);
}
extern "C"  float GetDistanceJointLength(void* joint) {
    b2DistanceJoint* m_joint = static_cast<b2DistanceJoint*>(joint);
    return m_joint->GetLength();
}
#pragma endregion

#pragma region RevoluteJoints
extern "C"  void* CreateRevoluteJoint(void* worldPointer, void* bodyA, void* bodyB, float anchorAX, float anchorAY, float anchorBX, float anchorBY, bool collideConnected) {
    b2World* world = static_cast<b2World*>(worldPointer);
    b2Body* m_bodyA = static_cast<b2Body*>(bodyA);
    b2Body* m_bodyB = static_cast<b2Body*>(bodyB);
    b2RevoluteJoint* rj;
    b2RevoluteJointDef jd;
    jd.bodyA = m_bodyA;
    jd.bodyB = m_bodyB;
    jd.localAnchorA.Set(anchorAX, anchorAY);
    jd.localAnchorB.Set(anchorBX, anchorBY);
    jd.collideConnected = collideConnected;
    rj = (b2RevoluteJoint*)world->CreateJoint(&jd);
    return static_cast<void*>(rj);
}
extern "C"  void SetRevoluteJointLimits(void* joint, float lower, float upper) {
    b2RevoluteJoint* m_joint = static_cast<b2RevoluteJoint*>(joint);
    m_joint->SetLimits(lower, upper);
}
extern "C"  void SetRevoluteJointMotorSpeed(void* joint, float speed) {
    b2RevoluteJoint* m_joint = static_cast<b2RevoluteJoint*>(joint);
    m_joint->SetMotorSpeed(speed);
}
extern "C"  void SetRevoluteJointMaxMotorTorque(void* joint, float torque) {
    b2RevoluteJoint* m_joint = static_cast<b2RevoluteJoint*>(joint);
    m_joint->SetMaxMotorTorque(torque);
}
extern "C"  void EnableRevoluteJointMotor(void* joint, bool motor) {
    b2RevoluteJoint* m_joint = static_cast<b2RevoluteJoint*>(joint);
    m_joint->EnableMotor(motor);
}
extern "C"  void EnableRevoluteJointLimits(void* joint, bool limit) {
    b2RevoluteJoint* m_joint = static_cast<b2RevoluteJoint*>(joint);
    m_joint->EnableLimit(limit);
}
extern "C"  float GetRevoluteJointUpperLimit(void* joint) {
    b2RevoluteJoint* m_joint = static_cast<b2RevoluteJoint*>(joint);
    return m_joint->GetUpperLimit();
}
extern "C"  float GetRevoluteJointLowerLimit(void* joint) {
    b2RevoluteJoint* m_joint = static_cast<b2RevoluteJoint*>(joint);
    return m_joint->GetLowerLimit();
}
extern "C"  bool IsRevoluteJointMotorEnabled(void* joint) {
    b2RevoluteJoint* m_joint = static_cast<b2RevoluteJoint*>(joint);
    return m_joint->IsMotorEnabled();
}
extern "C"  float GetRevoluteJointMotorSpeed(void* joint) {
    b2RevoluteJoint* m_joint = static_cast<b2RevoluteJoint*>(joint);
    return m_joint->GetMotorSpeed();
}
extern "C"  float GetRevoluteJointMotorTorque(void* joint, float invDt) {
    b2RevoluteJoint* m_joint = static_cast<b2RevoluteJoint*>(joint);
    return m_joint->GetMotorTorque(invDt);
}
extern "C"  float GetRevoluteJointMaxMotorTorque(void* joint) {
    b2RevoluteJoint* m_joint = static_cast<b2RevoluteJoint*>(joint);
    return m_joint->GetMaxMotorTorque();
}
#pragma endregion

#pragma region PrismaticJoints
extern "C"  void* CreatePrismaticJoint(void* worldPointer, void* bodyA, void* bodyB, float anchorAX, float anchorAY, float anchorBX, float anchorBY, float axisX, float axisY, bool collideConnect) {
    b2World* world = static_cast<b2World*>(worldPointer);
    b2Body* m_bodyA = static_cast<b2Body*>(bodyA);
    b2Body* m_bodyB = static_cast<b2Body*>(bodyB);
    b2PrismaticJoint* pj;
    b2PrismaticJointDef jd;
    jd.bodyA = m_bodyA;
    jd.bodyB = m_bodyB;
    jd.localAnchorA.Set(anchorAX, anchorAY);
    jd.localAnchorB.Set(anchorBX, anchorBY);
    jd.localAxisA.Set(axisX, axisY);
    jd.collideConnected = collideConnect;
    pj = (b2PrismaticJoint*)world->CreateJoint(&jd);
    return static_cast<void*>(pj);
}
extern "C"  void SetPrismaticJointLimits(void* joint, float lower, float upper) {
    b2PrismaticJoint* m_joint = static_cast<b2PrismaticJoint*>(joint);
    m_joint->SetLimits(lower, upper);
}
extern "C"  void SetPrismaticJointMotorSpeed(void* joint, float speed) {
    b2PrismaticJoint* m_joint = static_cast<b2PrismaticJoint*>(joint);
    m_joint->SetMotorSpeed(speed);
}
extern "C"  void SetPrismaticJointMaxMotorForce(void* joint, float force) {
    b2PrismaticJoint* m_joint = static_cast<b2PrismaticJoint*>(joint);
    m_joint->SetMaxMotorForce(force);
}
extern "C"  void EnablePrismaticJointMotor(void* joint, bool motor) {
    b2PrismaticJoint* m_joint = static_cast<b2PrismaticJoint*>(joint);
    m_joint->EnableMotor(motor);
}
extern "C"  void EnablePrismaticJointLimits(void* joint, bool limit) {
    b2PrismaticJoint* m_joint = static_cast<b2PrismaticJoint*>(joint);
    m_joint->EnableLimit(limit);
}
extern "C"  float GetPrismaticJointUpperLimit(void* joint) {
    b2PrismaticJoint* m_joint = static_cast<b2PrismaticJoint*>(joint);
    return m_joint->GetUpperLimit();
}
extern "C"  float GetPrismaticJointLowerLimit(void* joint) {
    b2PrismaticJoint* m_joint = static_cast<b2PrismaticJoint*>(joint);
    return m_joint->GetLowerLimit();
}
extern "C"  bool IsPrismaticJointMotorEnabled(void* joint) {
    b2PrismaticJoint* m_joint = static_cast<b2PrismaticJoint*>(joint);
    return m_joint->IsMotorEnabled();
}
extern "C"  float GetPrismaticJointMotorSpeed(void* joint) {
    b2PrismaticJoint* m_joint = static_cast<b2PrismaticJoint*>(joint);
    return m_joint->GetMotorSpeed();
}
extern "C"  float GetPrismaticJointMotorTorque(void* joint, float invDt) {
    b2PrismaticJoint* m_joint = static_cast<b2PrismaticJoint*>(joint);
    return m_joint->GetMotorForce(invDt);
}
extern "C"  float GetPrismaticJointMaxMotorForce(void* joint) {
    b2PrismaticJoint* m_joint = static_cast<b2PrismaticJoint*>(joint);
    return m_joint->GetMaxMotorForce();
}
extern "C"  float GetPrismaticJointMotorForce(void* joint,float its) {
    b2PrismaticJoint* m_joint = static_cast<b2PrismaticJoint*>(joint);
    return m_joint->GetMotorForce(its);
}

extern "C"  float GetPrismaticJointSpeed(void* joint) {
    b2PrismaticJoint* m_joint = static_cast<b2PrismaticJoint*>(joint);
    return m_joint->GetJointSpeed();
}
#pragma endregion

#pragma region PulleyJoints
extern "C"  void* CreatePulleyJoint(void* worldPointer, void* bodyA, void* bodyB, float groundAnchorAX, float groundAanchorAY, float groundAnchorBX, float groundAanchorBY, float anchorAX, float anchorAY, float anchorBX, float anchorBY, float ratio, float lengthA, float lengthB, bool collideConnect) {
    b2World* world = static_cast<b2World*>(worldPointer);
    b2Body* m_bodyA = static_cast<b2Body*>(bodyA);
    b2Body* m_bodyB = static_cast<b2Body*>(bodyB);
    b2PulleyJoint* pj;
    b2PulleyJointDef jd;
    jd.bodyA = m_bodyA;
    jd.bodyB = m_bodyB;
    jd.groundAnchorA.Set(groundAnchorAX, groundAanchorAY);
    jd.groundAnchorB.Set(groundAnchorBX, groundAanchorBY);
    jd.localAnchorA.Set(anchorAX, anchorAY);
    jd.localAnchorB.Set(anchorBX, anchorBY);
    jd.ratio = ratio;
    jd.collideConnected = collideConnect;
    jd.lengthA = lengthA;
    jd.lengthB = lengthB;
    pj = (b2PulleyJoint*)world->CreateJoint(&jd);
    return static_cast<void*>(pj);
}
extern "C"  float GetPulleyJointLengthA(void* joint) {
    b2PulleyJoint* m_joint = static_cast<b2PulleyJoint*>(joint);
    return m_joint->GetLengthA();
}
extern "C"  float GetPulleyJointLengthB(void* joint) {
    b2PulleyJoint* m_joint = static_cast<b2PulleyJoint*>(joint);
    return m_joint->GetLengthB();
}
#pragma endregion

#pragma region GearJoints
extern "C"  void* CreateGearJoint(void* worldPointer, void* bodyA, void* bodyB, void* jointA, bool isARevolute, void* jointB, bool isBRevolute, float ratio, bool collideConnect) {
    b2World* world = static_cast<b2World*>(worldPointer);
    b2Body* m_bodyA = static_cast<b2Body*>(bodyA);
    b2Body* m_bodyB = static_cast<b2Body*>(bodyB);
    if (isARevolute && isBRevolute) {
        b2RevoluteJoint* m_jointA = static_cast<b2RevoluteJoint*>(jointA);
        b2RevoluteJoint* m_jointB = static_cast<b2RevoluteJoint*>(jointB);
        b2GearJoint* gj;
        b2GearJointDef jd;
        jd.bodyA = m_bodyA;
        jd.bodyB = m_bodyB;
        jd.joint1 = m_jointA;
        jd.joint2 = m_jointB;
        jd.ratio = ratio;
        jd.collideConnected = collideConnect;
        gj = (b2GearJoint*)world->CreateJoint(&jd);
        return static_cast<void*>(gj);
    }
    else if (!isARevolute && !isBRevolute) {
        b2PrismaticJoint* m_jointA = static_cast<b2PrismaticJoint*>(jointA);
        b2PrismaticJoint* m_jointB = static_cast<b2PrismaticJoint*>(jointB);
        b2GearJoint* gj;
        b2GearJointDef jd;
        jd.bodyA = m_bodyA;
        jd.bodyB = m_bodyB;
        jd.joint1 = m_jointA;
        jd.joint2 = m_jointB;
        jd.ratio = ratio;
        jd.collideConnected = collideConnect;
        gj = (b2GearJoint*)world->CreateJoint(&jd);
        return static_cast<void*>(gj);
    }
    else {
        if (isARevolute && !isBRevolute) {
            b2RevoluteJoint* m_jointA = static_cast<b2RevoluteJoint*>(jointA);
            b2PrismaticJoint* m_jointB = static_cast<b2PrismaticJoint*>(jointB);
            b2GearJoint* gj;
            b2GearJointDef jd;
            jd.bodyA = m_bodyA;
            jd.bodyB = m_bodyB;
            jd.joint1 = m_jointA;
            jd.joint2 = m_jointB;
            jd.ratio = ratio;
            jd.collideConnected = collideConnect;
            gj = (b2GearJoint*)world->CreateJoint(&jd);
            return static_cast<void*>(gj);
        }
        else {
            b2PrismaticJoint* m_jointA = static_cast<b2PrismaticJoint*>(jointA);
            b2RevoluteJoint* m_jointB = static_cast<b2RevoluteJoint*>(jointB);
            b2GearJoint* gj;
            b2GearJointDef jd;
            jd.bodyA = m_bodyA;
            jd.bodyB = m_bodyB;
            jd.joint1 = m_jointA;
            jd.joint2 = m_jointB;
            jd.ratio = ratio;
            jd.collideConnected = collideConnect;
            gj = (b2GearJoint*)world->CreateJoint(&jd);
            return static_cast<void*>(gj);
        }
    }
}
extern "C"  void SetGearJointRatio(void* joint, float ratio) {
    b2GearJoint* m_joint = static_cast<b2GearJoint*>(joint);
    m_joint->SetRatio(ratio);
}
extern "C"  float GetGearJointRatio(void* joint) {
    b2GearJoint* m_joint = static_cast<b2GearJoint*>(joint);
    return m_joint->GetRatio();
}

#pragma endregion

#pragma region WheelJoints

extern "C"  void* CreateWheelJoint(void* worldPointer, void* bodyA, void* bodyB, float anchorAX, float anchorAY, float anchorBX, float anchorBY, float axisA, float axisB, bool collideConnect) {
    b2World* world = static_cast<b2World*>(worldPointer);
    b2Body* m_bodyA = static_cast<b2Body*>(bodyA);
    b2Body* m_bodyB = static_cast<b2Body*>(bodyB);
    b2WheelJoint* wj;
    b2WheelJointDef jd;
    jd.bodyA = m_bodyA;
    jd.bodyB = m_bodyB;
    jd.localAnchorA.Set(anchorAX, anchorAY);
    jd.localAnchorB.Set(anchorBX, anchorBY);
    jd.localAxisA.Set(axisA, axisB);
    jd.collideConnected = collideConnect;
    wj = (b2WheelJoint*)world->CreateJoint(&jd);
    return static_cast<void*>(wj);
}
extern "C"  void SetWheelJointSpringDampingRatio(void* joint, float ratio) {
    b2WheelJoint* m_joint = static_cast<b2WheelJoint*>(joint);
    m_joint->SetSpringDampingRatio(ratio);
}
extern "C"  float GetWheelJointSpringDampingRatio(void* joint) {
    b2WheelJoint* m_joint = static_cast<b2WheelJoint*>(joint);
    return m_joint->GetSpringDampingRatio();
}
extern "C"  void SetWheelJointSpringFrequency(void* joint, float frequency) {
    b2WheelJoint* m_joint = static_cast<b2WheelJoint*>(joint);
    m_joint->SetSpringFrequencyHz(frequency);
}
extern "C"  float GetWheelJointSpringFrequency(void* joint) {
    b2WheelJoint* m_joint = static_cast<b2WheelJoint*>(joint);
    return m_joint->GetSpringFrequencyHz();
}
extern "C"  void SetWheelJointMotorSpeed(void* joint, float speed) {
    b2WheelJoint* m_joint = static_cast<b2WheelJoint*>(joint);
    m_joint->SetMotorSpeed(speed);
}
extern "C"  void SetWheelJointMaxMotorTorque(void* joint, float torque) {
    b2WheelJoint* m_joint = static_cast<b2WheelJoint*>(joint);
    m_joint->SetMaxMotorTorque(torque);
}
extern "C"  void EnableWheelJointMotor(void* joint, bool motor) {
    b2WheelJoint* m_joint = static_cast<b2WheelJoint*>(joint);
    m_joint->EnableMotor(motor);
}
extern "C"  bool IsWheelJointMotorEnabled(void* joint) {
    b2WheelJoint* m_joint = static_cast<b2WheelJoint*>(joint);
    return m_joint->IsMotorEnabled();
}
extern "C"  float GetWheelJointMotorSpeed(void* joint) {
    b2WheelJoint* m_joint = static_cast<b2WheelJoint*>(joint);
    return m_joint->GetMotorSpeed();
}
extern "C"  float GetWheelJointMotorTorque(void* joint, float invDt) {
    b2WheelJoint* m_joint = static_cast<b2WheelJoint*>(joint);
    return m_joint->GetMotorTorque(invDt);
}
extern "C"  float GetWheelJointMaxMotorTorque(void* joint) {
    b2WheelJoint* m_joint = static_cast<b2WheelJoint*>(joint);
    return m_joint->GetMaxMotorTorque();
}

#pragma endregion

#pragma region WeldJoints

extern "C"  void* CreateWeldJoint(void* worldPointer, void* bodyA, void* bodyB, float anchorAX, float anchorAY, float anchorBX, float anchorBY) {
    b2World* world = static_cast<b2World*>(worldPointer);
    b2Body* m_bodyA = static_cast<b2Body*>(bodyA);
    b2Body* m_bodyB = static_cast<b2Body*>(bodyB);
    b2WeldJoint* wj;
    b2WeldJointDef jd;
    jd.bodyA = m_bodyA;
    jd.bodyB = m_bodyB;
    jd.localAnchorA.Set(anchorAX, anchorAY);
    jd.localAnchorB.Set(anchorBX, anchorBY);
    wj = (b2WeldJoint*)world->CreateJoint(&jd);
    return static_cast<void*>(wj);
}
extern "C"  float GetWeldJointFrequency(void* joint) {
    b2WeldJoint* m_joint = static_cast<b2WeldJoint*>(joint);
    return m_joint->GetFrequency();
}
extern "C"  float GetWeldJointDampingRatio(void* joint) {
    b2WeldJoint* m_joint = static_cast<b2WeldJoint*>(joint);
    return m_joint->GetDampingRatio();
}
extern "C"  void SetWeldJointFrequency(void* joint, float frequency) {
    b2WeldJoint* m_joint = static_cast<b2WeldJoint*>(joint);
    m_joint->SetFrequency(frequency);
}
extern "C"  void SetWeldJointDampingRatio(void* joint, float ratio) {
    b2WeldJoint* m_joint = static_cast<b2WeldJoint*>(joint);
    m_joint->SetDampingRatio(ratio);
}
#pragma endregion

#pragma region FrictionJoints

extern "C"  void* CreateFrictionJoint(void* worldPointer, void* bodyA, void* bodyB, float anchorAX, float anchorAY, float anchorBX, float anchorBY, bool collideConnect) {
    b2World* world = static_cast<b2World*>(worldPointer);
    b2Body* m_bodyA = static_cast<b2Body*>(bodyA);
    b2Body* m_bodyB = static_cast<b2Body*>(bodyB);
    b2FrictionJoint* fj;
    b2FrictionJointDef jd;
    jd.bodyA = m_bodyA;
    jd.bodyB = m_bodyB;
    jd.localAnchorA.Set(anchorAX, anchorAY);
    jd.localAnchorB.Set(anchorBX, anchorBY);
    jd.collideConnected = collideConnect;
    fj = (b2FrictionJoint*)world->CreateJoint(&jd);
    return static_cast<void*>(fj);
}
extern "C"  float GetFrictionJointMaxForce(void* joint) {
    b2FrictionJoint* m_joint = static_cast<b2FrictionJoint*>(joint);
    return m_joint->GetMaxForce();
}
extern "C"  float GetFrictionJointMaxTorque(void* joint) {
    b2FrictionJoint* m_joint = static_cast<b2FrictionJoint*>(joint);
    return m_joint->GetMaxTorque();
}
extern "C"  void SetFrictionJointMaxForce(void* joint, float force) {
    b2FrictionJoint* m_joint = static_cast<b2FrictionJoint*>(joint);
    m_joint->SetMaxForce(force);
}
extern "C"  void SetFrictionJointMaxTorque(void* joint, float torque) {
    b2FrictionJoint* m_joint = static_cast<b2FrictionJoint*>(joint);
    m_joint->SetMaxTorque(torque);
}

#pragma endregion

#pragma region RopeJoints

extern "C"  void* CreateRopeJoint(void* worldPointer, void* bodyA, void* bodyB, float anchorAX, float anchorAY, float anchorBX, float anchorBY, float maxLength, bool collideConnect) {
    b2World* world = static_cast<b2World*>(worldPointer);
    b2Body* m_bodyA = static_cast<b2Body*>(bodyA);
    b2Body* m_bodyB = static_cast<b2Body*>(bodyB);
    b2RopeJoint* rj;
    b2RopeJointDef jd;
    jd.bodyA = m_bodyA;
    jd.bodyB = m_bodyB;
    jd.localAnchorA.Set(anchorAX, anchorAY);
    jd.localAnchorB.Set(anchorBX, anchorBY);
    jd.maxLength = maxLength;
    jd.collideConnected = collideConnect;
    rj = (b2RopeJoint*)world->CreateJoint(&jd);
    return static_cast<void*>(rj);
}
extern "C"  float GetRopeJointMaxLength(void* joint) {
    b2RopeJoint* m_joint = static_cast<b2RopeJoint*>(joint);
    return m_joint->GetMaxLength();
}
extern "C"  void SetRopeJointMaxLength(void* joint, float length) {
    b2RopeJoint* m_joint = static_cast<b2RopeJoint*>(joint);
    m_joint->SetMaxLength(length);
}

#pragma endregion

#pragma region MouseJoints

extern "C"  void* CreateMouseJoint(void* worldPointer, void* bodyA, void* bodyB, float targetX, float targetY, bool collideConnect) {
    b2World* world = static_cast<b2World*>(worldPointer);
    b2Body* m_bodyA = static_cast<b2Body*>(bodyA);
    b2Body* m_bodyB = static_cast<b2Body*>(bodyB);
    b2Vec2 target = b2Vec2(targetX, targetY);
    b2MouseJoint* rj;
    b2MouseJointDef jd;
    jd.bodyA = m_bodyA;
    jd.bodyB = m_bodyB;
    jd.target = target;
    jd.collideConnected = collideConnect;
    rj = (b2MouseJoint*)world->CreateJoint(&jd);
    return static_cast<void*>(rj);
}
extern "C"  float GetMouseJointFrequency(void* joint) {
    b2MouseJoint* m_joint = static_cast<b2MouseJoint*>(joint);
    return m_joint->GetFrequency();
}
extern "C"  float GetMouseJointMaxForce(void* joint) {
    b2MouseJoint* m_joint = static_cast<b2MouseJoint*>(joint);
    return m_joint->GetMaxForce();
}
extern "C"  float GetMouseJointDampingRatio(void* joint) {
    b2MouseJoint* m_joint = static_cast<b2MouseJoint*>(joint);
    return m_joint->GetDampingRatio();
}
extern "C"  void SetMouseJointFrequency(void* joint, float frequency) {
    b2MouseJoint* m_joint = static_cast<b2MouseJoint*>(joint);
    m_joint->SetFrequency(frequency);
}
extern "C"  void SetMouseJointMaxForce(void* joint, float maxForce) {
    b2MouseJoint* m_joint = static_cast<b2MouseJoint*>(joint);
    m_joint->SetMaxForce(maxForce);
}
extern "C"  void SetMouseJointDampingRatio(void* joint, float dampingRatio) {
    b2MouseJoint* m_joint = static_cast<b2MouseJoint*>(joint);
    m_joint->SetDampingRatio(dampingRatio);
}
extern "C"  void SetMouseJointTarget(void* joint, float targetX, float targetY) {
    b2MouseJoint* m_joint = static_cast<b2MouseJoint*>(joint);
    b2Vec2 target = b2Vec2(targetX, targetY);
    m_joint->SetTarget(target);
}

#pragma endregion

#pragma region GenericFunctions
extern "C"  void DeleteJoint(void* worldPointer, void* jointPointer) {
    b2World* world = static_cast<b2World*>(worldPointer);
    b2Joint* joint = static_cast<b2Joint*>(jointPointer);
    world->DestroyJoint(joint);
}
extern "C"  bool GetJointCollideConnected(void* jointPointer) {
    b2Joint* joint = static_cast<b2Joint*>(jointPointer);
    return joint->GetCollideConnected();
}
extern "C"  void ShiftJointOrigin(void* joint, float x, float y) {
    b2Joint* m_joint = static_cast<b2Joint*>(joint);
    b2Vec2 origin = b2Vec2(x, y);
    m_joint->ShiftOrigin(origin);
}
#pragma endregion

#pragma endregion

#pragma region Raycasting

extern "C"  float* RaycastWorld(void* world, float x1, float y1, float x2, float y2, int mode, bool shouldQuery) {
    b2World* m_world = static_cast<b2World*>(world);
    newRC = new b2NewRaycastCallback(mode, shouldQuery);
    b2Vec2 pos1 = b2Vec2(x1, y1);
    b2Vec2 pos2 = b2Vec2(x2, y2);
    m_world->RayCast(newRC, pos1, pos2);
    return newRC->GetData();
}

#pragma endregion

#pragma region MemoryReleasing

extern "C"  int ReleaseFloatArray(float* floatArray)
{
    delete[] floatArray;
    return 0;
}

extern "C"  int ReleaseIntArray(int* intArray)
{
    delete[] intArray;
    return 0;
}

extern "C"  int ReleaseShape(b2Shape* shape)
{
    delete shape;
    return 0;
}

#pragma endregion

#pragma region test

extern "C"  int TestInt()
{
    return 114;
}

#pragma endregion