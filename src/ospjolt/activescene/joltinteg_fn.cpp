/**
 * Open Space Program
 * Copyright © 2019-2020 Open Space Program Project
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "joltinteg_fn.h"          // IWYU pragma: associated
#include <osp/activescene/basic_fn.h>

#include <utility>                   // for std::exchange
#include <cassert>                   // for assert

// IWYU pragma: no_include <cstddef>
// IWYU pragma: no_include <type_traits>

using namespace ospjolt;

// for the 0xrrggbb_rgbf and angle literals
using namespace Magnum::Math::Literals;

using osp::EShape;

using osp::active::ActiveEnt;
using osp::active::ACtxPhysics;
using osp::active::SysSceneGraph;

using osp::Matrix3;
using osp::Matrix4;
using osp::Vector3;

void SysJolt::resize_body_data(ACtxJoltWorld& rCtxWorld)
{
    std::size_t const capacity = rCtxWorld.m_bodyIds.capacity();
    rCtxWorld.m_ospToJoltBodyId.resize(capacity);
    rCtxWorld.m_bodyToEnt      .resize(capacity);
    rCtxWorld.m_bodyFactors    .resize(capacity);
}


void SysJolt::update_translate(ACtxPhysics& rCtxPhys, ACtxJoltWorld& rCtxWorld) noexcept
{

    // Origin translation
    if (Vector3 const translate = std::exchange(rCtxPhys.m_originTranslate, {});
        ! translate.isZero())
    {
        PhysicsSystem* pJoltWorld = rCtxWorld.m_world.get();
        BodyIDVector allBodiesIds;
        pJoltWorld->GetBodies(allBodiesIds);

        BodyInterface &bodyInterface = pJoltWorld->GetBodyInterface();

        // Translate every jolt body
        for (JoltBodyId bodyId : allBodiesIds)
        {
            RVec3 position = bodyInterface.GetPosition(bodyId);
            Matrix4 matrix;
            position += RVec3(translate.x(), translate.y(), translate.z());
            //As we are translating the whole world, we don't need to wake up asleep bodies. 
            bodyInterface.SetPosition(bodyId, position, EActivation::DontActivate);
        }
    }
}

using Corrade::Containers::ArrayView;

void SysJolt::update_world(
        ACtxPhysics&                rCtxPhys,
        ACtxJoltWorld&              rCtxWorld,
        float                       timestep,
        ACompTransformStorage_t&    rTf) noexcept
{
    PhysicsSystem *pJoltWorld = rCtxWorld.m_world.get();
    BodyInterface &bodyInterface = pJoltWorld->GetBodyInterface();

    // Apply changed velocities
    for (auto const& [ent, vel] : std::exchange(rCtxPhys.m_setVelocity, {}))
    {
        OspBodyId const ospBodyId     = rCtxWorld.m_entToBody.at(ent);
        JoltBodyId const bodyId     = rCtxWorld.m_ospToJoltBodyId[ospBodyId];

        bodyInterface.SetLinearVelocity(bodyId, Vec3(vel.x(), vel.y(), vel.z()));
    }

    rCtxWorld.m_pTransform = std::addressof(rTf);

    // Update the world
    pJoltWorld->Update(timestep, 1, &rCtxWorld.m_temp_allocator, rCtxWorld.m_joltJobSystem.get());
}

void SysJolt::remove_components(ACtxJoltWorld& rCtxWorld, ActiveEnt ent) noexcept
{
    auto itBodyId = rCtxWorld.m_entToBody.find(ent);

    if (itBodyId != rCtxWorld.m_entToBody.end())
    {
        OspBodyId const bodyId = itBodyId->second;
        rCtxWorld.m_bodyIds.remove(bodyId);
        rCtxWorld.m_bodyToEnt[bodyId] = lgrn::id_null<ActiveEnt>();
        rCtxWorld.m_entToBody.erase(itBodyId);
    }

}

TransformedShapePtr_t SysJolt::create_primitive(ACtxJoltWorld &rCtxWorld, osp::EShape shape)
{
    Shape* joltShape;
    switch (shape)
    {
    case EShape::Sphere:
        joltShape = (Shape *) new SphereShape(1.0f);
        break;
    case EShape::Box:
        joltShape = (Shape *) new BoxShape(Vec3Arg(1.0f, 1.0f, 1.0f));
        break;
    case EShape::Cylinder:
        //cylinder needs to be internally rotated 90° to match with graphics
        joltShape = (Shape *) new CylinderShape(1.0f, 2.0f);
        joltShape = (Shape *) new RotatedTranslatedShape(Vec3Arg::sZero(), Quat::sRotation(Vec3::sAxisX(), JPH_PI/2), joltShape);
        break;
    default:
        // TODO: support other shapes, sphere is used for now
        joltShape = (Shape *) new SphereShape(1.0f);
        break;
    }
    return std::make_unique<TransformedShape>(RVec3::sZero(), Quat::sZero(), joltShape, BodyID(), SubShapeIDCreator());
}

void SysJolt::orient_shape(TransformedShapePtr_t& pJoltShape, osp::EShape ospShape, osp::Vector3 const &translation, osp::Matrix3 const &rotation, osp::Vector3 const &scale)
{
    auto rawQuat = Magnum::Quaternion::fromMatrix(rotation).data();
    Quat joltRotation(rawQuat[0], rawQuat[1], rawQuat[2], rawQuat[3]);

    //scale the shape directly
    pJoltShape->mShape = pJoltShape->mShape->ScaleShape(Vec3Arg(scale.x(), scale.y(), scale.z())).Get();
    
    pJoltShape->SetWorldTransform(RVec3Arg(translation.x(), translation.y(), translation.z()), joltRotation, Vec3Arg(1.0f, 1.0f, 1.0f));
    
}

float SysJolt::get_inverse_mass_no_lock(PhysicsSystem &physicsSystem, JoltBodyId joltBodyId)
{
    const BodyLockInterfaceNoLock& lockInterface = physicsSystem.GetBodyLockInterfaceNoLock(); 
    {
        JPH::BodyLockRead lock(lockInterface, joltBodyId);
        if (lock.Succeeded()) // body_id may no longer be valid
        {
            const JPH::Body &body = lock.GetBody();

            return body.GetMotionProperties()->GetInverseMass();
        }
    }
    return 0.0f;
}

void SysJolt::find_shapes_recurse(
        ACtxPhysics const&                      rCtxPhys,
        ACtxJoltWorld&                          rCtxWorld,
        ACtxSceneGraph const&                   rScnGraph,
        ACompTransformStorage_t const&          rTf,
        ActiveEnt                               ent,
        Matrix4 const&                          transform,
        CompoundShapeSettings&                  pCompound) noexcept
{
    // Add jolt shape if exists
    if (rCtxWorld.m_shapes.contains(ent))
    {
        TransformedShapePtr_t& pShape = rCtxWorld.m_shapes.get(ent);

        // Set transform relative to root body

        SysJolt::orient_shape(pShape, rCtxPhys.m_shape[ent], transform.translation(), transform.rotation(), transform.scaling());
        Ref<Shape> rScaledShape = pShape->mShape->ScaleShape(Vec3(pShape->mShapeScale)).Get();
        pCompound.AddShape(pShape->mShapePositionCOM, pShape->mShapeRotation, rScaledShape);
    }

    if ( ! rCtxPhys.m_hasColliders.contains(ent) )
    {
        return;
    }

    // Recurse into children if there are more shapes
    for (ActiveEnt child : SysSceneGraph::children(rScnGraph, ent))
    {
        if (rTf.contains(child))
        {
            ACompTransform const &rChildTransform = rTf.get(child);

            Matrix4 const childMatrix = transform * rChildTransform.m_transform;

            find_shapes_recurse(
                    rCtxPhys, rCtxWorld, rScnGraph, rTf, child, childMatrix, pCompound);
        }

    }

}

//TODO this is locking on all bodies. Is it bad ?
//The easy fix is to provide multiple step listeners for disjoint sets of bodies, which can then run in parallel.
//It might not be worth it considering this function should be quite fast.
void PhysicsStepListenerImpl::OnStep(float inDeltaTime, PhysicsSystem &rJoltWorld)
{
    //no lock as all bodies are already locked
    BodyInterface &bodyInterface = rJoltWorld.GetBodyInterfaceNoLock();
    for (OspBodyId ospBody : m_context->m_bodyIds)
    {
        JoltBodyId joltBody = m_context->m_ospToJoltBodyId[ospBody];
        if (bodyInterface.GetMotionType(joltBody) != EMotionType::Dynamic) 
        {
            continue;
        }

        //Transform jolt -> osp

        ActiveEnt const ent = m_context->m_bodyToEnt[ospBody];
        Mat44 worldTranform = bodyInterface.GetWorldTransform(joltBody);

        worldTranform.StoreFloat4x4((Float4*)m_context->m_pTransform->get(ent).m_transform.data());

        //Force and torque osp -> jolt
        Vector3 force{0.0f};
        Vector3 torque{0.0f};

        auto factorBits = lgrn::bit_view(m_context->m_bodyFactors[ospBody]);
        for (std::size_t const factorIdx : factorBits.ones())
        {   
            ACtxJoltWorld::ForceFactorFunc const& factor = m_context->m_factors[factorIdx];
            factor.m_func(joltBody, ospBody, *m_context, factor.m_userData, force, torque);
        }
        Vec3 vel = bodyInterface.GetLinearVelocity(joltBody);

        bodyInterface.AddForceAndTorque(joltBody, Vec3Arg (force.x(), force.y(), force.z()), Vec3Arg(torque.x(), torque.y(), torque.z()));
    }
}