#pragma once
#include <Box2D/Box2D.h>

struct Contact
{
    b2Body* bodyA;
    b2Body* bodyB;

};

typedef void (*BeginContactCallback)(Contact);
typedef void (*EndContactCallback)(Contact);

class UnityContactListener : public b2ContactListener
{
public:
    BeginContactCallback beginCallback;
    EndContactCallback endCallback;


    UnityContactListener(BeginContactCallback beginCallback, EndContactCallback endCallback);
    void BeginContact(b2Contact* contact);
    void EndContact(b2Contact* contact);
    void PreSolve(b2Contact* contact, const b2Manifold* oldManifold);
    void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse);
 };