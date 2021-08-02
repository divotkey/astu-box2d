/*
 * ASTU/Box2D
 * An integration of Erin Catto's 2D Physics Engine to AST-Utilities.
 * 
 * Copyright (c) 2020, 2021 Roman Divotkey. All rights reserved.
 */

#pragma once

// AST-Utilities includes
#include <Suite2D/CBody.h>

 // Forward declaration
class b2Body;

namespace astu::suite2d {

    class CBox2DBody : public CBody {
    public:

        /**
         * Constructor.
         */
        CBox2DBody()
            : boxBody(nullptr)
        {
            // Intentionally left empty.
        }

        // Inherited via CBody
        virtual std::shared_ptr<EntityComponent> Clone() override {
            // Create copy using copy-constructor.
            return std::make_shared<CBox2DBody>(*this);
        }

        virtual void OnAddedToEntity(Entity & entity) override
        {
            entity.AddInterface(*this, typeid(CBody));
        }

        virtual void SetType(CBody::Type bodyType) override;
        virtual Vector2f GetLinearVelocity() const override;
        virtual CBody& SetLinearVelocity(float vx, float vy) override;
        virtual float GetAngularVelocity() const override;
        virtual CBody& SetAngularVelocity(float av) override;
        virtual void SetLinearDamping(float damping) override;
        virtual void SetAngularDamping(float damping) override;
        virtual void ApplyTorque(float torque) override;
        virtual Vector2f GetWorldVector(float lvx, float lvy) override;
        virtual Vector2f GetWorldPoint(float lpx, float lpy) override;
        virtual Vector2f GetLocalVector(float wvx, float wvy) override;
        virtual Vector2f GetLocalPoint(float wpx, float wpy) override;
        virtual void ApplyForce(const Vector2f& force) override;

    private:
        /** The actual Box2D body. */
        b2Body* boxBody;

        friend class Box2DPhysicsSystem;
    };

} // end of namespace