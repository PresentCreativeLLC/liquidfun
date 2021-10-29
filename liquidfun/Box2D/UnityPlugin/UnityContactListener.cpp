#include "UnityContactListener.h"

#define NOT_USED(x) ((void)(x))

UnityContactListener::UnityContactListener(BeginContactCallback beginCallback, EndContactCallback endCallback):
	beginCallback(beginCallback),
	endCallback(endCallback)
{

}

void UnityContactListener::BeginContact(b2Contact* contact)
{
	Contact contacts;
	contacts.bodyA = contact->GetFixtureA()->GetBody();
	contacts.bodyB = contact->GetFixtureB()->GetBody();
	beginCallback(contacts);
}

void UnityContactListener::EndContact(b2Contact* contact)
{
	Contact contacts;
	contacts.bodyA = contact->GetFixtureA()->GetBody();
	contacts.bodyB = contact->GetFixtureB()->GetBody();
	endCallback(contacts);
}

void UnityContactListener::PreSolve(b2Contact* contact, const b2Manifold* oldManifold)
{
	NOT_USED(contact);
	NOT_USED(oldManifold);
}

void UnityContactListener::PostSolve(b2Contact* contact, const b2ContactImpulse* impulse)
{
	NOT_USED(contact);
	NOT_USED(impulse);
}