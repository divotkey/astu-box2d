/*
 * ASTU/Box2D
 * An integration of Erin Catto's 2D Physics Engine to AST-Utilities.
 * 
 * Copyright (c) 2020, 2021 Roman Divotkey. All rights reserved.
 */

#pragma once

// AST-Utilities includes
#include <Service/Service.h>
#include <Service/UpdateService.h>
#include <Service/TimeService.h>
#include <ECS/EntitySystems.h>
#include <Suite2D/PhysicsSystem.h>
#include <Suite2D/CBody.h>
#include <Suite2D/CColliders.h>
#include <Suite2D/CollisionSignal.h>


// C++ Standard Library includes.
#include <memory>

 // forward declaration
class b2World;
class b2Body;

namespace astu::suite2d {

    // Forward declaration
    class ContactListener;

    class Box2DPhysicsSystem 
        : public BaseService
        , private OneFamilyEntitySystem
        , private EntityListener
        , private TimeClient
        , private Updatable
        , public PhysicsSystem
        , public CBodyFactory
        , public CCircleColliderFactory
        , public CPolygonColliderFactory
    {
    public:

        /**
         * Constructor.
         * 
         * @param updatePriority    the priority used to update this system
         */
        Box2DPhysicsSystem(int updatePriority = Priority::Normal);

        /**
         * Virtual destructor.
         */
        virtual ~Box2DPhysicsSystem();

        /**
         * Sets the number of iterations for the position phase constraint solver.
         * 
         * @param iterations the number of position iterations
         * @return reference to this system for method chaining
         */
        Box2DPhysicsSystem& SetPositionIterations(int iterations);

        /**
         * Returns the number of iterations for the position phase constraint solver.
         * 
         * @return the number of position iterations
         */
        int GetPositionIterations() const {
            return positionIterations;
        }

        /**
         * Sets the number of iterations for the velocity phase constraint solver.
         * 
         * @param iterations the number of velocity iterations
         * @return reference to this system for method chaining
         */
        Box2DPhysicsSystem& SetVelocityIterations(int iterations);

        /**
         * Returns the number of iterations for the velocity phase constraint solver.
         * 
         * @return the number of velocity iterations
         */
        int GetVelocityIterations() const {
            return velocityIterations;
        }


        /**
         * Returns the gravity vector used for physics simulation.
         * 
         * @return the gravity vector
         */
        const Vector2f &GetGravity() const {
            return gravity;
        }

        // Inherited via PhysicsSystem
        virtual PhysicsSystem& SetGravityVector(float gx, float gy) override;
        virtual const Vector2f& GetGravityVector() const override;

        // Inherited via CBodyFactory
        virtual std::shared_ptr<CBody> CreateBody() override;

        // Inherited via CCircleColliderFactory
        virtual std::shared_ptr<CCircleCollider> CreateCircleCollider() override;

        // Inherited via CPolygonColliderFactory
        virtual std::shared_ptr<CPolygonCollider> CreatePolygonCollider() override;

    private:
        /** The family of entities this system processes. */
        static const EntityFamily FAMILY;

		/** The Box2D physics island. */
		std::unique_ptr<b2World> world;

		/** Number of iterations for the velocity phase constraint solver. */
		int velocityIterations;

		/** Number of iterations for the position phase constraint solver. */
		int positionIterations;

        /** The gravity vector. */
        Vector2f gravity;

        /** Used to receive contacts from Box2d. */
        std::unique_ptr<ContactListener> contactListener;

        /** Used to publish collision events. */
        std::shared_ptr<CollisionSignalService> collisionSignals;

        /**
         * Creates fixtures according to the collider components of the entity.
         * 
         * @param entity    the entity 
         * @param body      the Box2D body
         */
        void AddFixture(Entity& entity, b2Body& body);

        void CollectTransforms();
        void DeployTransforms();
        void HandleCollision(std::shared_ptr<Entity> a, std::shared_ptr<Entity> b);

        // Inherited via BaseService
        virtual void OnStartup() override;
        virtual void OnShutdown() override;

        // Inherited via Updatable
        virtual void OnUpdate() override;

        // Inherited via EntityListener
        virtual void OnEntityAdded(std::shared_ptr<Entity> entity) override;
        virtual void OnEntityRemoved(std::shared_ptr<Entity> entity) override;

        friend class ContactListener;
    };

} // end of namespace