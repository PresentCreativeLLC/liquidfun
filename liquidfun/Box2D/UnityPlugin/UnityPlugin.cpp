

#include <Box2D/Box2D.h>
#include <stdio.h>

#include "UnityContactListener.h"

#ifdef APPLE
#define DllExport
#else
#define DllExport _declspec(dllexport)
#endif

#define FALSE 0
#define TRUE 1

/*
 * The following structs are used to commnunicate with unity
*/

/**
* @brief  Vector of two dimensions
*/
struct Vec2 {
	float x, y;
};


/**
* @brief  Represents the position and angle of an object
*/
struct BodyTransform {
	b2Vec2 position;
	float angle;
};

/**
* @brief  Definition paramaters of a body
*/
struct BodyDef {
	b2BodyType bodyType;
	b2Vec2 position;
	float angle;
	float linearDamping;
	float angularDamping;
	float gravityScale;
	int allowSleepFlag;
	int awakeFlag;
	int fixedRotationFlag;
	int isSensorFlag;
	float density;
	float friction;
	float restitution;
	int filterGroup;
};

/**
* @brief  Definition paramaters of a circle shape
*/
struct CircleShapeDef {
	float radius;
};

/**
* @brief  Definition paramaters of a box shape
*/
struct BoxShapeDef {
	float hx;
	float hy;
};

/**
* @brief  Definition paramaters of a polygon shape
*/
struct PolygonShapeDef {
	int count;
	b2Vec2* vertices;
};

/**
* @brief  Definition paramaters of a chain shape
*/
struct ChainShapeDef {
	int count;
	b2Vec2* vertices;
	int isLoop;
};


/**
* @brief  Helper function to create a body object definition
*/
b2BodyDef BuildBodyDef(BodyDef& definition) {
	b2BodyDef bodyDef;
	bodyDef.type = definition.bodyType;
	bodyDef.position = definition.position;
	bodyDef.angle = definition.angle;
	bodyDef.linearDamping = definition.linearDamping;
	bodyDef.angularDamping = definition.angularDamping;
	bodyDef.gravityScale = definition.gravityScale;
	bodyDef.fixedRotation = definition.fixedRotationFlag == TRUE;
	bodyDef.allowSleep = definition.allowSleepFlag == TRUE;
	bodyDef.awake = definition.awakeFlag == TRUE;

	return bodyDef;
}

/**
* @brief  Helper function to create a ficture definition
*/
b2FixtureDef BuildFixtureDef(BodyDef definition, b2Shape* shape)
{
	b2FixtureDef fixtureDef;
	fixtureDef.shape = shape;
	fixtureDef.isSensor = definition.isSensorFlag == TRUE;
	fixtureDef.density = definition.density;
	fixtureDef.friction = definition.friction;
	fixtureDef.restitution = definition.restitution;
	fixtureDef.filter.groupIndex = (short)definition.filterGroup;
	
	return fixtureDef;
}


/**
 * Functions inside the  extern C block are visible and usable by unity c# context.
 * More information about liquid fun here https://google.github.io/liquidfun/Programmers-Guide/html/index.html
 */
extern "C" 
{

	/**
	 * @brief Create a world pointer and return it
	 */
	DllExport b2World* CreateWorld(float gravityX, float gravityY)
	{
		// Define the gravity vector.
		b2Vec2 gravity(gravityX, gravityY);

		// Construct a world object, which will hold and simulate the rigid bodies.
		b2World* world = new b2World(gravity);
		return world;
	}

	/**
	 * @brief Delete a world pointer from memory
	 */
	DllExport void DeleteWorld(b2World* world)
	{
		if (world == nullptr)
			return;
		delete world;
	}

	/**
	 * @brief  Create a contact listener and attach it to the given world pointer
	 */
	DllExport UnityContactListener* CreateContactListener(
		b2World* world, 
		BeginContactCallback beginCallback, 
		EndContactCallback endCallback)
	{
		UnityContactListener* listener = new UnityContactListener(beginCallback, endCallback);
		world->SetContactListener(listener);
		return listener;
	}

	/**
	 * @brief  Delete a contact listener from memory
	 */
	DllExport void DeleteContactListener(UnityContactListener* listener)
	{
		if (listener == nullptr)
			return;
		delete listener;
	}

	/**
	 * @brief  Create a body and attach a circle shape to it
	 */
	DllExport b2Body* CreateCircleBody(b2World* world, BodyDef definition, CircleShapeDef shape) {

		if (world == nullptr)
			return nullptr;

		b2BodyDef bodyDef = BuildBodyDef(definition);

		b2Body* body = world->CreateBody(&bodyDef);
		b2CircleShape circleShape;
		circleShape.SetPosition(0.0f, 0.0f);
		circleShape.m_radius = shape.radius;

		b2FixtureDef fixtureDef = BuildFixtureDef(definition, &circleShape);
		body->CreateFixture(&fixtureDef);

		return body;
	}


	/**
	 * @brief  Create a body and attach a box shape to it
	 */
	DllExport b2Body* CreateBoxBody(b2World* world, BodyDef definition, BoxShapeDef shape) {

		if (world == nullptr)
			return nullptr;

		b2BodyDef bodyDef = BuildBodyDef(definition);

		b2Body* body = world->CreateBody(&bodyDef);
		b2PolygonShape polygonShape;
		polygonShape.SetAsBox(shape.hx, shape.hy);

		b2FixtureDef fixtureDef = BuildFixtureDef(definition, &polygonShape);
		body->CreateFixture(&fixtureDef);

		return body;
	}


	/**
	 * @brief  Create a body and attach a polygon shape to it
	 */
	DllExport b2Body* CreatePolygonBody(b2World* world, BodyDef definition, PolygonShapeDef shape) {

		if (world == nullptr)
			return nullptr;

		b2BodyDef bodyDef = BuildBodyDef(definition);
		bodyDef.fixedRotation = false;

		b2Body* body = world->CreateBody(&bodyDef);
		b2PolygonShape polygonShape;
		polygonShape.Set(shape.vertices, shape.count);

		b2FixtureDef fixtureDef = BuildFixtureDef(definition, &polygonShape);
		body->CreateFixture(&fixtureDef);

		return body;
	}

	/**
	 * @brief  Create a body and attach a chain shape to it
	 */
	DllExport b2Body* CreateChainBody(b2World* world, BodyDef definition, ChainShapeDef shape) {

		if (world == nullptr)
			return nullptr;

		b2BodyDef bodyDef = BuildBodyDef(definition);
		bodyDef.fixedRotation = false;

		b2Body* body = world->CreateBody(&bodyDef);
		b2ChainShape chainShape;
		if (shape.isLoop)
		{
			chainShape.CreateLoop(shape.vertices, shape.count);
		}
		else
		{
			chainShape.CreateChain(shape.vertices, shape.count);
		}
		

		b2FixtureDef fixtureDef = BuildFixtureDef(definition, &chainShape);
		body->CreateFixture(&fixtureDef);
	
		return body;
	}

	/**
	 * @brief  Create a body and all shapes inside it
	 */
	DllExport void DestroyBody(b2World* world, b2Body* body) {
		if (world == nullptr || body == nullptr)
			return;

		world->DestroyBody(body);
	}

	/**
	 * @brief  Execute one step of the phicis engine
	 */
	DllExport void Step(b2World* world, float timestep, int velocityIterations, int positionIterations) {
		if (world == nullptr)
			return;
		world->Step(timestep, velocityIterations, positionIterations);
		world->ClearForces();
	}


	/**
	 * @brief  Get the transform of the given body
	 */
	DllExport BodyTransform GetTransform(b2Body* body) {

		BodyTransform transform;

		if (body == nullptr)
			return transform;
		
		transform.position = body->GetPosition();
		transform.angle = body->GetAngle();

		return transform;
	}

	/**
	 * @brief  set the transform to the given object
	 */
	DllExport void SetTransform(b2Body* body, BodyTransform transform) {
		body->SetTransform(transform.position, transform.angle);
	}

	/**
	 * @brief  Get the center position of the given body in world coordinates
	 */
	DllExport Vec2 GetWorldCenter(b2Body* body) {
		 
		const b2Vec2& value = body->GetWorldCenter();

		Vec2 vec;
		vec.x = value.x;
		vec.y = value.y;
		return vec;
	}

	/**
	 * @brief  Get the center position of the given body in local coordinates
	 */
	DllExport Vec2 GetLocalCenter(b2Body* body) {

		const b2Vec2& value = body->GetLocalCenter();

		Vec2 vec;
		vec.x = value.x;
		vec.y = value.y;
		return vec;
	}


	/**
	 * @brief  Set the linear velocity of a body
	 */
	DllExport void SetLinearVelocity(b2Body* body, b2Vec2 v) {
		body->SetLinearVelocity(v);
	}


	/**
	 * @brief  Get the linear velocity the given body
	 */
	DllExport Vec2 GetLinearVelocity(b2Body* body) {

		const b2Vec2& value = body->GetLinearVelocity();

		Vec2 vec;
		vec.x = value.x;
		vec.y = value.y;
		return vec;
	}

	/**
	 * @brief  Set the angular velocity of the given body
	 */
	DllExport void SetAngularVelocity(b2Body* body, float omega) {
		body->SetAngularVelocity(omega);
	}

	/**
	 * @brief  Get the linear velocity of a given body
	 */
	DllExport float GetAngularVelocity(b2Body* body) {
		return body->GetAngularVelocity();
	}

	/**
	 * @brief  Apply a force in the given point to the given body
	 */
	DllExport void ApplyForce(b2Body* body, b2Vec2 force, b2Vec2 point, int wake) {
		body->ApplyForce(force, point, wake);
	}

	/**
	 * @brief  Apply a force to the center of the given object
	 */
	DllExport void ApplyForceToCenter(b2Body* body, b2Vec2 force, int wake) {
		body->ApplyForceToCenter(force, wake);
	}

	/**
	 * @brief  Apply a force in the given point to the given body
	 */
	DllExport void ApplyTorque(b2Body* body, float torque, int wake) {
		body->ApplyTorque(torque, wake);
	}

	/**
	 * @brief  Apply impulse to the given body
	 */
	DllExport void ApplyLinearImpulse(b2Body* body, b2Vec2 impulse, b2Vec2 point, int wake) {
		body->ApplyLinearImpulse(impulse, point, wake);
	}

	/**
	 * @brief  Apply angular impulse to the given body
	 */
	DllExport void ApplyAngularImpulse(b2Body* body, float impulse, int wake) {
		body->ApplyAngularImpulse(impulse, wake);
	}

	/**
	 * @brief  Set the damping of an body
	 */
	DllExport void SetLinearDamping(b2Body* body, float linearDamping) {
		body->SetLinearDamping(linearDamping);
	}


	/**
	 * @brief  Set the anguar damping of a given body
	 */
	DllExport void SetAngularDamping(b2Body* body, float angularDamping) {
		body->SetAngularDamping(angularDamping);
	}


	/**
	 * @brief  Set the gravity scale for the given body
	 */
	DllExport void SetGravityScale(b2Body* body, float scale) {
		body->SetGravityScale(scale);
	}

	/**
	 * @brief  Set the body type of the given body
	 */
	DllExport void SetType(b2Body* body, b2BodyType type) {
		body->SetType(type);
	}

	/**
	 * @brief  Set sleeping allowed property of the given body
	 */
	DllExport void SetSleepingAllowed(b2Body* body, int flag) {
		body->SetSleepingAllowed(flag == TRUE);
	}

	/**
	 * @brief  Set awake property of the given body
	 */
	DllExport void SetAwake(b2Body* body, int flag) {
		body->SetAwake(flag == TRUE);
	}

	/**
	 * @brief  Set active the given body
	 */
	DllExport void SetActive(b2Body* body, int flag) {
		body->SetActive(flag == TRUE);
	}

	/**
	 * @brief  Set if the given obejct has a fixed rotation
	 */
	DllExport void SetFixedRotation(b2Body* body, int flag) {
		body->SetFixedRotation(flag == TRUE);
	}

	/**
	 * @brief  Set the filter group of the given body
	 */
	DllExport void SetFilterGroup(b2Body* body, int group) {
		b2Fixture* fixture = body->GetFixtureList();
		b2Filter filter = fixture->GetFilterData();
		filter.groupIndex = (short)group;
		fixture->SetFilterData(filter);
		
	}


	/**
	 * @brief Set the gravity of the given world
	 */
	DllExport void SetWorldGravity(b2World* world, Vec2 gravity) {
		world->SetGravity(gravity.x, gravity.y);
	}

	/**
	 * @brief  Set the density of the given body
	 */
	DllExport void SetDensity(b2Body* body, float density) {
		b2Fixture* fixture = body->GetFixtureList();
		fixture->SetDensity(density);
	}

	/**
	 * @brief  Set the friction of the given body
	 */
	DllExport void SetFriction(b2Body* body, float friction) {
		b2Fixture* fixture = body->GetFixtureList();
		fixture->SetFriction(friction);
	}

	/**
	 * @brief  Set the restitution (bouncing) of the given body
	 */
	DllExport void SetRestitution(b2Body* body, float restitution) {
		b2Fixture* fixture = body->GetFixtureList();
		fixture->SetRestitution(restitution);
	}

	/**
	 * @brief  Set if the given body is a sensor
	 */
	DllExport void SetSensor(b2Body* body, int isSensor) {
		b2Fixture* fixture = body->GetFixtureList();
		fixture->SetSensor(isSensor == TRUE);
	}
}

