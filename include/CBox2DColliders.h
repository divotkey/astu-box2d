/*
 * ASTU/Box2D
 * An integration of Erin Catto's 2D Physics Engine to AST-Utilities.
 * 
 * Copyright (c) 2020, 2021 Roman Divotkey. All rights reserved.
 */

#pragma once

// AST-Utilities includes
#include <Suite2D/CColliders.h>

// Box2D includes
#include <box2d/b2_fixture.h>

// C++ Standard Library includes
#include <vector>

// Forward declaration
class b2Body;

namespace astu::suite2d {

    /**
     * Interface for Box2D colliders.
     */
    class IBox2DCollider {
    public:

        /** Virtual destructor. */
        virtual ~IBox2DCollider() {}

        /**
         * Creates a Box2D fixture for the specified body.
         * 
         * @param body  the body for which to create the fixture
         */
        virtual void CreateFixture(b2Body & body) = 0; 
    };

    template <typename T>
    class CBox2DBaseCollider : public T, public IBox2DCollider {
    public:

        /**
         * Constructor.
         */
        CBox2DBaseCollider()
            : fixture(nullptr)
        {
            // Intentionally left empty            
        }

        /** Virtual destructor. */
        virtual ~CBox2DBaseCollider() {}

        // Inherited via T
        virtual void SetRestitution(float r) override {
            T::SetRestitution(r);
            if (fixture) {
                fixture->SetRestitution(r);
            }
        }

        virtual void SetFriction(float f) override {
            T::SetFriction(f);
            if (fixture) {
                fixture->SetFriction(f);
            }
        }

        virtual void SetDensity(float d) override {
            T::SetDensity(d);
            if (fixture) {
                fixture->SetDensity(d);
            }
        }

        virtual void SetCategoryBits(uint16_t bits) override {
            T::SetCategoryBits(bits);
            if (fixture) {
                auto filterData = fixture->GetFilterData();
                filterData.categoryBits = bits;
                fixture->SetFilterData(filterData);
            }
        }

        virtual void SetMaskBits(uint16_t bits) override {
            T::SetMaskBits(bits);
            if (fixture) {
                auto filterData = fixture->GetFilterData();
                filterData.maskBits = bits;
                fixture->SetFilterData(filterData);
            }
        }

    protected:
        /** The Box2D fixture. */
        b2Fixture* fixture;

        void ConfigureFixtureDef(b2FixtureDef& fixtureDef) {
            fixtureDef.restitution = T::GetRestitution();
            fixtureDef.friction = T::GetFriction();
            fixtureDef.density = T::GetDensity();
            fixtureDef.filter.categoryBits = T::GetCategoryBits();
            fixtureDef.filter.maskBits = T::GetMaskBits();
            fixtureDef.userData.pointer = reinterpret_cast<uintptr_t>(this);
        }
    };

    class CBox2DCircleCollider : public CBox2DBaseCollider<CCircleCollider> {
    public:

        /**
         * Constructor.
         */
        CBox2DCircleCollider()
        {
            // Intentionally left empty.
        }

        // Inherited via CCircleCollider
        virtual std::shared_ptr<EntityComponent> Clone() override {
            // Create copy using copy-constructor.
            return std::make_shared<CBox2DCircleCollider>(*this);
        }

        // Inherited via IBox2DCollider
        virtual void CreateFixture(b2Body & body) override;
    };

    class CBox2DPolygonCollider : public CBox2DBaseCollider<CPolygonCollider> {
    public:

        /**
         * Constructor.
         */
        CBox2DPolygonCollider()
        {
            // Intentionally left empty.
        }

        // Inherited via CCircleCollider
        virtual std::shared_ptr<EntityComponent> Clone() override {
            // Create copy using copy-constructor.
            return std::make_shared<CBox2DPolygonCollider>(*this);
        }

        // Inherited via IBox2DCollider
        virtual void CreateFixture(b2Body & body) override;

    private:
        /** Used to create Box2D polygon shapes. */
        static std::vector<b2Vec2> tempVertices;
    };


} // end of namespace