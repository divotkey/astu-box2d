/*
 * ASTU/Box2D
 * An integration of Erin Catto's 2D Physics Engine to AST-Utilities.
 * 
 * Copyright (c) 2020, 2021 Roman Divotkey. All rights reserved.
 */

// Local includes
#include "Box2DPhysicsSystem.h"
#include "CBox2DBody.h"
#include "CBox2DColliders.h"

// AST-Utilities includes
#include <Suite2D/CPose.h>

// Box2D includes
#include <box2d/box2d.h>

// C++ Standard Libraries includes
#include <iostream>

using namespace std;

namespace astu::suite2d {

    class ContactListener : public b2ContactListener
    {
    public:

        ContactListener(Box2DPhysicsSystem& context)
            : context(context)
        {
            // Intentionally left empty.
        }

        // Inherited via b2ContactListener
        virtual void BeginContact(b2Contact* contact) override { 
            auto entityA = reinterpret_cast<EntityComponent*>(
                contact->GetFixtureA()->GetUserData().pointer
            );

            auto entityB = reinterpret_cast<EntityComponent*>(
                contact->GetFixtureB()->GetUserData().pointer
            );

            context.HandleCollision(entityA->GetParent(), entityB->GetParent());
        }

        virtual void EndContact(b2Contact* contact) override { 
            // Intentionally left empty.
        }

    private:
        /** The context of this listener. */
        Box2DPhysicsSystem& context;
    };

    const EntityFamily Box2DPhysicsSystem::FAMILY = EntityFamily::Create<CBox2DBody, CPose>();

    Box2DPhysicsSystem::Box2DPhysicsSystem()
        : BaseService("Box2D Physics System")
        , OneFamilyEntitySystem(FAMILY)
        , EntityListener(FAMILY)
        , velocityIterations(8)
        , positionIterations(3)
        , gravity(0, 0)
        , contactListener(make_unique<ContactListener>(*this))
    {
        // Intentionally left empty.
    }

    Box2DPhysicsSystem::~Box2DPhysicsSystem()
    {
        // Intentionally left empty.
    }

    Box2DPhysicsSystem& Box2DPhysicsSystem::SetPositionIterations(int iterations)
    {
        if (iterations <= 0) {
            throw std::logic_error("Position iterations must be greater zero");
        }
        positionIterations = iterations;
        return *this;
    }

    Box2DPhysicsSystem& Box2DPhysicsSystem::SetVelocityIterations(int iterations)
    {
        if (iterations <= 0) {
            throw std::logic_error("Velocity iterations must be greater zero");
        }
        velocityIterations = iterations;
        return *this;
    }

    PhysicsSystem& Box2DPhysicsSystem::SetGravityVector(float gx, float gy) {
        gravity.Set(gx, gy);
        if (IsStarted() && world) {
            world->SetGravity(b2Vec2(gravity.x, gravity.y));
        }
        return *this;
    }
    
    const Vector2f& Box2DPhysicsSystem::GetGravityVector() const
    {
        return gravity;
    }

    shared_ptr<CBody> Box2DPhysicsSystem::CreateBody()
    {
        return make_shared<CBox2DBody>();
    }

    shared_ptr<CCircleCollider> Box2DPhysicsSystem::CreateCircleCollider()
    {
        return make_shared<CBox2DCircleCollider>();
    }

    shared_ptr<CPolygonCollider> Box2DPhysicsSystem::CreatePolygonCollider()
    {
        return make_shared<CBox2DPolygonCollider>();
    }

    void Box2DPhysicsSystem::HandleCollision(std::shared_ptr<Entity> a, std::shared_ptr<Entity> b)
    {
        if (collisionSignals) {
            collisionSignals->QueueSignal(CollisionSignal(a, b));
        }
    }

    void Box2DPhysicsSystem::OnStartup() 
    {
        // Create physics world.
        world = make_unique<b2World>(b2Vec2(gravity.x, gravity.y));
        world->SetContactListener(contactListener.get());

        collisionSignals = ASTU_GET_SERVICE_OR_NULL(CollisionSignalService);
    }

    void Box2DPhysicsSystem::OnShutdown()
    {
        // Release resources.
        collisionSignals = nullptr;
        world = nullptr;
    }

    void Box2DPhysicsSystem::OnUpdate()
    {
        CollectTransforms();
        world->Step(GetElapsedTimeF(), velocityIterations, positionIterations);
        DeployTransforms();
    }
    
    void Box2DPhysicsSystem::CollectTransforms()
    {
        for (auto & entity : GetEntityView()) {
            
            auto& body = entity->GetComponent<CBox2DBody>();
            if (body.GetType() == CBody::Type::Kinematic) {
                const auto& tx = entity->GetComponent<CPose>().transform;
                body.boxBody->SetTransform(
                    b2Vec2(tx.GetTranslationX(), tx.GetTranslationY()), 
                    tx.GetRotation()
                );
            }

        }
    }

    void Box2DPhysicsSystem::DeployTransforms()
    {
        for (auto & entity : GetEntityView()) {

            auto& body = entity->GetComponent<CBox2DBody>();
            if (body.GetType() == CBody::Type::Dynamic) {
                auto& pose = entity->GetComponent<CPose>();
                const b2Vec2& p = body.boxBody->GetPosition();
                pose.transform.SetTranslation(p.x, p.y);
                pose.transform.SetRotation(body.boxBody->GetAngle());
            }
        }
    }

    void Box2DPhysicsSystem::OnEntityAdded(shared_ptr<Entity> entity)
    {
        auto& pose = entity->GetComponent<CPose>();
        auto& body = entity->GetComponent<CBox2DBody>();

		b2BodyDef bodyDef;

		switch (body.GetType()) {
		case CBody::Type::Static:
			bodyDef.type = b2_staticBody;
			break;
		case CBody::Type::Kinematic:
			bodyDef.type = b2_kinematicBody;
			break;
		case CBody::Type::Dynamic:
			bodyDef.type = b2_dynamicBody;
			break;
		}

        bodyDef.position.Set(pose.transform.GetTranslationX(), pose.transform.GetTranslationY());
        bodyDef.angle = pose.transform.GetRotation();
        bodyDef.linearDamping = body.GetLinearDamping();
        bodyDef.angularDamping = body.GetAngularDamping();
        bodyDef.linearVelocity.Set(body.GetLinearVelocity().x, body.GetLinearVelocity().y);
        bodyDef.angularVelocity = body.GetAngularVelocity();
        bodyDef.fixedRotation = false;
        body.boxBody = world->CreateBody(&bodyDef);

        // b2MassData massData;
        // body.boxBody->GetMassData(&massData);
        // if (massData.mass == 0) {
        //     massData.mass = 1.0f;
        // }
        // body.boxBody->SetMassData(&massData);

        AddFixture(*entity, *body.boxBody);
    }

    void Box2DPhysicsSystem::OnEntityRemoved(shared_ptr<Entity> entity)
    {
        // Destroy the Box2D body.
        auto& body = entity->GetComponent<CBox2DBody>();
        assert(body.boxBody);
        world->DestroyBody(body.boxBody);
        body.boxBody = nullptr;
    }

    void Box2DPhysicsSystem::AddFixture(Entity& entity, b2Body& body)
    {
        if (entity.HasComponent<CBodyCollider>()) {
            auto& col = entity.GetComponent<CBodyCollider>();    
            IBox2DCollider* boxCol = dynamic_cast<IBox2DCollider*>(&col);
            if (boxCol) {
                boxCol->CreateFixture(body);        
            }
        }
    }


} // end of namespace

