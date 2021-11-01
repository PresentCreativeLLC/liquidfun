

#include <Box2D/Box2D.h>
#include <stdio.h>

#include "UnityContactListener.h"

#define DllExport _declspec(dllexport)

#define FALSE 0
#define TRUE 1

struct Vec2 {
	float x, y;
};

struct BodyTransform {
	b2Vec2 position;
	float angle;
};

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
};

struct CircleShapeDef {
	float radius;
};

struct BoxShapeDef {
	float hx;
	float hy;
};

struct PolygonShapeDef {
	int count;
	b2Vec2* vertices;
};

struct ChainShapeDef {
	int count;
	b2Vec2* vertices;
	int isLoop;
};

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

b2FixtureDef BuildFixtureDef(BodyDef definition, b2Shape* shape) 
{
	b2FixtureDef fixtureDef;
	fixtureDef.shape = shape;
	fixtureDef.isSensor = definition.isSensorFlag == TRUE;
	fixtureDef.density = definition.density;
	fixtureDef.friction = definition.friction;
	fixtureDef.restitution = definition.restitution;
	
	return fixtureDef;
}

extern "C" 
{

	DllExport b2World* CreateWorld(float gravityX, float gravityY)
	{
		// Define the gravity vector.
		b2Vec2 gravity(gravityX, gravityY);

		// Construct a world object, which will hold and simulate the rigid bodies.
		b2World* world = new b2World(gravity);
		return world;
	}

	DllExport void DeleteWorld(b2World* world)
	{
		if (world == nullptr)
			return;
		delete world;
	}

	DllExport UnityContactListener* CreateContactListener(
		b2World* world, 
		BeginContactCallback beginCallback, 
		EndContactCallback endCallback)
	{
		UnityContactListener* listener = new UnityContactListener(beginCallback, endCallback);
		world->SetContactListener(listener);
		return listener;
	}

	DllExport void DeleteContactListener(UnityContactListener* listener)
	{
		if (listener == nullptr)
			return;
		delete listener;
	}


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

	DllExport void DestroyBody(b2World* world, b2Body* body) {
		if (world == nullptr || body == nullptr)
			return;

		world->DestroyBody(body);
	}

	DllExport void Step(b2World* world, float timestep, int velocityIterations, int positionIterations) {
		if (world == nullptr)
			return;
		world->Step(timestep, velocityIterations, positionIterations);
		world->ClearForces();
	}

	DllExport BodyTransform GetTransform(b2Body* body) {

		BodyTransform transform;

		if (body == nullptr)
			return transform;
		
		transform.position = body->GetPosition();
		transform.angle = body->GetAngle();

		return transform;
	}

	DllExport void SetTransform(b2Body* body, BodyTransform transform) {
		body->SetTransform(transform.position, transform.angle);
	}

	DllExport Vec2 GetWorldCenter(b2Body* body) {
		 
		const b2Vec2& value = body->GetWorldCenter();

		Vec2 vec;
		vec.x = value.x;
		vec.y = value.y;
		return vec;
	}

	DllExport Vec2 GetLocalCenter(b2Body* body) {

		const b2Vec2& value = body->GetLocalCenter();

		Vec2 vec;
		vec.x = value.x;
		vec.y = value.y;
		return vec;
	}

	DllExport void SetLinearVelocity(b2Body* body, b2Vec2 v) {
		body->SetLinearVelocity(v);
	}

	DllExport Vec2 GetLinearVelocity(b2Body* body) {

		const b2Vec2& value = body->GetLinearVelocity();

		Vec2 vec;
		vec.x = value.x;
		vec.y = value.y;
		return vec;
	}

	DllExport void SetAngularVelocity(b2Body* body, float omega) {
		body->SetAngularVelocity(omega);
	}

	DllExport float GetAngularVelocity(b2Body* body) {
		return body->GetAngularVelocity();
	}

	DllExport void ApplyForce(b2Body* body, b2Vec2 force, b2Vec2 point, int wake) {
		return body->ApplyForce(force, point, wake);
	}

	DllExport void ApplyForceToCenter(b2Body* body, b2Vec2 force, int wake) {
		return body->ApplyForceToCenter(force, wake);
	}

	DllExport void ApplyTorque(b2Body* body, float torque, int wake) {
		return body->ApplyTorque(torque, wake);
	}

	DllExport void ApplyLinearImpulse(b2Body* body, b2Vec2 impulse, b2Vec2 point, int wake) {
		return body->ApplyLinearImpulse(impulse, point, wake);
	}

	DllExport void ApplyAngularImpulse(b2Body* body, float impulse, int wake) {
		return body->ApplyAngularImpulse(impulse, wake);
	}

	DllExport void SetLinearDamping(b2Body* body, float linearDamping) {
		return body->SetLinearDamping(linearDamping);
	}

	DllExport void SetAngularDamping(b2Body* body, float angularDamping) {
		return body->SetAngularDamping(angularDamping);
	}

	DllExport void SetGravityScale(b2Body* body, float scale) {
		return body->SetGravityScale(scale);
	}

	DllExport void SetType(b2Body* body, b2BodyType type) {
		return body->SetType(type);
	}

	DllExport void SetSleepingAllowed(b2Body* body, int flag) {
		return body->SetSleepingAllowed(flag == TRUE);
	}

	DllExport void SetAwake(b2Body* body, int flag) {
		return body->SetAwake(flag == TRUE);
	}

	DllExport void SetActive(b2Body* body, int flag) {
		return body->SetActive(flag == TRUE);
	}

	DllExport void SetFixedRotation(b2Body* body, int flag) {
		return body->SetFixedRotation(flag == TRUE);
	}
}

