/******************************************************************************\
* Copyright (C) Leap Motion, Inc. 2011-2013.                                   *
* Leap Motion proprietary and  confidential.  Not for distribution.            *
* Use subject to the terms of the Leap Motion SDK Agreement available at       *
* https://developer.leapmotion.com/sdk_agreement, or another agreement between *
* Leap Motion and you, your company or other organization.                     *
\******************************************************************************/
#include "LeapScene.h"
#include <cfloat>
#include <cstring>

namespace Leap {

using namespace LeapUtil;

#if defined(LEAP_SCENE_USE_UTIL_GL)
  using namespace LeapUtilGL;
#endif

//***********************
//
// Scene public methods
//
//***********************

Scene::Scene()
  : m_pUserData( NULL ),
    m_fDeltaTimeSeconds( 0.0f ),
    m_fPointableRadius( 1.0f ),
    m_fSelectHitTime( 1.0f ),
    m_uiNumObjects( 0 ),
    m_uiNumRayHits( 0 ),
    m_uiNumQueuedInteractions( 0 ),
    m_uiNumPendingRemovals( 0 ),
    m_uiNextSerial( 0 ),
    m_uiFlags( kF_UpdateRayCast | kF_UpdateContact )
{
  memset( m_apObjects, 0, sizeof(m_apObjects) );
}

void Scene::RemoveObject( SceneObject* pObject )
{
  if ( pObject && (pObject->m_pScene == this) && !pObject->m_bPendingRemoval )
  {
    pObject->m_bPendingRemoval = true;
    m_uiNumPendingRemovals++;
  }
}

void Scene::Reset()
{
  for ( uint32_t i = 0; i < m_uiNumObjects; i++ )
  {
    if ( SceneObject* pObj = m_apObjects[i] )
    {
      pObj->m_pScene = NULL;
      pObj->m_index = kMaxObjects;
    }

    m_apObjects[i].Release();
  }

  m_uiNumObjects          = 0;
  m_uiNumPendingRemovals  = 0;

  clearInteractionQueue();
  clearRayHits();
}

void Scene::Update(const Frame& frame, float fDeltaTimeSeconds)
{
  m_fDeltaTimeSeconds = fDeltaTimeSeconds;

  clearRayHits();

  clearInteractionQueue();

  processPendingRemovals();

  updateSelectionAndContact( frame );

  updateInteraction( frame );
}

void Scene::DeselectAll()
{
  SceneInteraction deselection;
  deselection.m_uiFlags = kIT_SelectionChange;

  for (size_t i=0; i < m_uiNumObjects; i++)
  {
    m_apObjects[i]->SetSelected( false );
  }
}

const SceneObjectPtr& Scene::TestRayHit( const SceneRay& ray ) const
{
  float fClosestHitDist = FLT_MAX;
  int closestHitIndex   = -1;

  for (uint32_t i = 0; i < m_uiNumObjects; i++)
  {
    float fHitDist = FLT_MAX;
    if (m_apObjects[i]->TestRayHit(ray, fHitDist) && (fHitDist < fClosestHitDist) )
    {
      closestHitIndex = static_cast<int>(i);
      fClosestHitDist = fHitDist;
    }
  }

  if (closestHitIndex >= 0)
  {
    return m_apObjects[closestHitIndex];
  }

  return SceneObjectPtr::Null();
}

#if defined(LEAP_SCENE_USE_UTIL_GL)
void Scene::RayHitsDebugDrawGL() const
{
    GLAttribScope attribScope( GL_CURRENT_BIT | GL_LIGHTING_BIT );

    glDisable( GL_LIGHTING );
    glColor3f(1, 1, 1);

    const float fHitSphereSize = m_fPointableRadius * 0.3f;

    for ( uint32_t i = 0, e = GetNumRayHits(); i < e; i++ )
    {
      const SceneRayHit&  rayHit  = m_aRayHits[i];
      rayHit.DebugDrawGL( fHitSphereSize );
    }
}
#endif // LEAP_SCENE_USE_UTIL_GL

//***********************
//
// Scene internal methods
//
//***********************

void Scene::deallocateObject( uint32_t idxToRemove )
{
  if ( SceneObject* pObjToRemove = (idxToRemove < m_uiNumObjects) ? m_apObjects[idxToRemove].GetPointer() : NULL )
  {
    m_uiNumObjects--;

    // orphan the object
    pObjToRemove->m_pScene  = NULL;

    // assign an invalid index.
    pObjToRemove->m_index   = static_cast<uint32_t>(kMaxObjects);

    // if removing any object but the last one
    if ( idxToRemove != m_uiNumObjects )
    {
      // move the last object to the vacated slot - releases the reference to the outbound object.
      // increases the reference count on the the last object.
      m_apObjects[idxToRemove] = m_apObjects[m_uiNumObjects];

      // update its index to match
      m_apObjects[idxToRemove]->m_index = static_cast<uint16_t>(idxToRemove);

      // release the extra reference to the last object
      m_apObjects[m_uiNumObjects].Release();
    }
    else
    {
      // it was the last object - just release the reference to it.
      m_apObjects[idxToRemove].Release();
    }

  }
}

bool Scene::testRayHitClosest( SceneRayHit& hitResult )
{
  float fClosestHitDist = FLT_MAX;
  int closestHitIndex   = -1;

  for (uint32_t i = 0; i < m_uiNumObjects; i++)
  {
    float fHitDist = FLT_MAX;
    if (m_apObjects[i]->TestRayHit(hitResult.m_ray, fHitDist) && (fHitDist < fClosestHitDist) )
    {
      closestHitIndex = static_cast<int>(i);
      fClosestHitDist = fHitDist;
    }
  }

  if (closestHitIndex >= 0)
  {
    hitResult.m_fHitDistance  = fClosestHitDist;
    hitResult.m_pHitObject    = m_apObjects[closestHitIndex];
    hitResult.m_hitPoint      = hitResult.m_ray.CalcPointOn( hitResult.m_fHitDistance );

    m_apObjects[closestHitIndex]->IncNumPointing();

    m_apObjects[closestHitIndex]->m_fTotalHitTime += m_fDeltaTimeSeconds;

    return true;
  }

  return false;
}

void Scene::updateContact( const SceneContactPoint& testPoint )
{
  for (size_t i=0; i < m_uiNumObjects; i++)
  {
    if ( m_apObjects[i]->TestSphereHit( testPoint.m_vPoint, m_fPointableRadius ) )
    {
      m_apObjects[i]->IncNumContacts( testPoint );
      m_apObjects[i]->m_fTotalHitTime += m_fDeltaTimeSeconds;
    }
  }
}

void Scene::updateSelectionAndContact( const Frame& frame )
{
  const PointableList& pointables = frame.pointables();

  if ( pointables.isEmpty() && frame.hands().isEmpty() )
  {
    queueDeselectAll();
  }

  for ( uint32_t j = 0, numPointables = pointables.count(); j < numPointables; j++ )
  {
    const Pointable& pointable    = pointables[j];
    const Vector  vPos            = TransformFramePoint( pointable.tipPosition() );
    const int     iPointableID    = pointable.id();

    if ( GetUpdateRayCast() && (m_uiNumRayHits < static_cast<uint32_t>(kMaxRayHits)) )
    {
      SceneRayHit&  rayHit    = m_aRayHits[m_uiNumRayHits];

      rayHit.m_ray            = SceneRay( vPos, TransformFrameDirection( pointable.direction() ) );
      rayHit.m_iPointableID   = iPointableID;

      if ( testRayHitClosest( rayHit ) )
      {
        m_uiNumRayHits++;
      }
    }

    if ( GetUpdateContact() )
    {
      updateContact( SceneContactPoint( vPos, iPointableID ) );
    }
  }
}

void Scene::updateInteraction( const Frame& frame )
{
  static const uint8_t kMaxContactMissedFrames = 64;

  (void)frame;

  for (size_t i=0; i < m_uiNumObjects; i++)
  {
    const SceneObjectPtr& pObj = m_apObjects[i];

    if (!pObj->HasInitialContact() && pObj->GetNumContacts() == 0 && pObj->GetNumPointing() == 0)
    {
      pObj->m_fTotalHitTime = 0.0f;
    }
    /// selection state change from not selected to selected
    else if ( !pObj->IsSelected() && pObj->m_fTotalHitTime >= m_fSelectHitTime )
    {
      SceneInteraction selection;
      selection.m_uiFlags = kIT_SelectionChange | kIT_IsSelected;
      selection.m_pObject = pObj;
      queueInteraction( selection );
    }
    /// possible manipulation
    else if ( pObj->IsSelected() )
    {

      if ( pObj->HasInitialContact() )
      {
        const SceneContactPoint& initContact  = pObj->m_initialContactPoint;
        Leap::Pointable contactPointable      = frame.pointable(initContact.m_iPointableID);

        if ( contactPointable.isValid() )
        {
            const Vector      vPointablePos  = TransformFramePoint(contactPointable.tipPosition());
            const Vector      vTrans         = (vPointablePos - pObj->GetCenter());
            SceneInteraction  translation;

            translation.m_mtxTransform.origin = vTrans;
            translation.m_uiFlags |= kIT_Translation;
            translation.m_pObject = pObj;

            queueInteraction( translation );

            pObj->m_uiHasInitialContact = kMaxContactMissedFrames;
        }
        else
        {
          pObj->m_uiHasInitialContact--;

          if ( !pObj->m_uiLastNumContacts )
          {
            pObj->ClearHits();
          }
        }
      }
      else
      {
        if ( pObj->GetNumContacts()  )
        {
          pObj->m_initialContactPoint = *(pObj->GetContactPoint(0));
          pObj->m_initialContactPoint.m_vPoint -= pObj->GetCenter();
          pObj->m_uiHasInitialContact = kMaxContactMissedFrames;
        }
        else if ( pObj->GetLastNumContacts() )
        {
          pObj->m_initialContactPoint = *(pObj->GetLastContactPoint(0));
          pObj->m_initialContactPoint.m_vPoint -= pObj->GetCenter();
          pObj->m_uiHasInitialContact = kMaxContactMissedFrames;
        }
      }

      /// multi touch
      if (pObj->GetLastNumContacts() >= 2 && pObj->GetNumContacts() >= 2)
      {
        const SceneContactPoint&  lastContact0  = *(pObj->GetLastContactPoint(0));
        const SceneContactPoint&  lastContact1  = *(pObj->GetLastContactPoint(1));
        const SceneContactPoint*  pCurContact0  = pObj->GetContactPointByPointableID( lastContact0.m_iPointableID );
        const SceneContactPoint*  pCurContact1  = pObj->GetContactPointByPointableID( lastContact1.m_iPointableID );

        if ( pCurContact0 && pCurContact1 )
        {
          const Vector lastVec = (lastContact1.m_vPoint - lastContact0.m_vPoint);
          const Vector newVec = (pCurContact1->m_vPoint - pCurContact0->m_vPoint);

          if ( !IsNearEqual(lastVec, newVec) && !IsNearZero(lastVec) )
          {
            const float   fLastDist = lastVec.magnitude();
            const float   fNewDist  = newVec.magnitude();
            /// scale by change in pinch distance
            const float   fScale    = fNewDist/fLastDist;

            /// rotate by tilt of fingers
            const Vector  vLastDir  = lastVec * 1.0f/fLastDist;
            const Vector  vNewDir   = newVec  * 1.0f/fNewDist;

            const Vector  vAxis     = vNewDir.cross(vLastDir);
            const float   fAngle    = acosf( vNewDir.dot(vLastDir) );

            /// translate by average movement
            const Vector  vTrans    = ( (pCurContact0->m_vPoint - lastContact0.m_vPoint) +
                                        (pCurContact1->m_vPoint - lastContact1.m_vPoint) ) * 0.5f;

            SceneInteraction interaction;

            if ( !IsNearZero(fAngle) )
            {
              interaction.m_mtxTransform.setRotation( vAxis, fAngle );
              interaction.m_uiFlags |= kIT_Rotation;
            }

            if ( !IsNearEqual(fScale, 1.0f) )
            {
              interaction.m_fScale = fScale;
              interaction.m_uiFlags |= kIT_Scale;
            }

            if ( !IsNearZero( vTrans ) )
            {
              interaction.m_mtxTransform.origin = vTrans;
              interaction.m_uiFlags |= kIT_Translation;
            }

            if ( interaction.m_uiFlags )
            {
              interaction.m_pObject = pObj;
              queueInteraction( interaction );
            }
          }
        }
      }
    }

    pObj->rotateContactPoints();
    pObj->m_uiNumPointing = 0;
  }
}

void Scene::processPendingRemovals()
{
  for ( uint32_t i = 0; (i < m_uiNumObjects) && m_uiNumPendingRemovals; m_uiNumPendingRemovals-- )
  {
    if ( m_apObjects[i]->m_bPendingRemoval )
    {
      deallocateObject( i );
    }
    else
    {
      i++;
    }
  }
}

void Scene::queueDeselectAll()
{
  SceneInteraction deselection;
  deselection.m_uiFlags = kIT_SelectionChange;

  for (size_t i=0; i < m_uiNumObjects; i++)
  {
    if ( m_apObjects[i]->IsSelected() )
    {
      deselection.m_pObject = m_apObjects[i];
      queueInteraction( deselection );
    }
  }
}

//************************************
//
// SceneRayHit methods
//
//************************************

#if defined(LEAP_SCENE_USE_UTIL_GL)
void SceneRayHit::DebugDrawGL( float fHitSphereSize ) const
{
    const Vector&       pos     = m_ray.m_vOrigin;
    const Vector&       pt      = m_hitPoint;

    glBegin(GL_LINES);
    glVertex3fv( &(pos.x) );
    glVertex3fv( &(pt.x) );
    glEnd();

    {
      GLMatrixScope matrixScope;

      glTranslatef(pt.x, pt.y, pt.z);
      glScalef( fHitSphereSize, fHitSphereSize, fHitSphereSize );

      drawSphere( kStyle_Solid );
    }
}
#endif // LEAP_SCENE_USE_UTIL_GL

//************************************
//
// SceneObject methods
//
//************************************

bool SceneBox::TestRayHit(const SceneRay& testRay, float& fHitDistOut) const
{
  // by converting the test ray to object space the test is vs. an axis-aligned
  // box centered at the origin (0, 0, 0)

  SceneRay  testRayObj  = testRay.Transformed( GetWorldToObjectTransform() );
  Vector    vMax        = GetSize() * m_fScale * 0.5f;
  Vector    vMin        = -vMax;
  Vector    vDirScale   = ComponentWiseReciprocal(testRayObj.m_vDirection);
  Vector    vLower      = ComponentWiseScale(vMin - testRayObj.m_vOrigin, vDirScale);
  Vector    vUpper      = ComponentWiseScale(vMax - testRayObj.m_vOrigin, vDirScale);

  float fTMin           = MaxComponent(ComponentWiseMin(vLower, vUpper));
  float fTMax           = MinComponent(ComponentWiseMax(vLower, vUpper));

  if (fTMin < fTMax)
  {
    if (fTMin > 0)
    {
      fHitDistOut = fTMin;
      return true;
    }

    if (fTMax > 0)
    {
      fHitDistOut = fTMax;
      return true;
    }
  }

  return false;
}

bool SceneBox::TestSphereHit(const Vector& vTestPoint, float fTestRadius) const
{
  // by converting the test point to object space it's a sphere vs. axis-aligned box test
  // where the box is centered at the origin (0, 0, 0).
  Vector vPos       = WorldToObjectPoint(vTestPoint);
  Vector vMax       = GetSize() * m_fScale * 0.5f;
  Vector vMin       = -vMax;

  return  ( ComponentWiseMin(vPos - vMin, Vector::zero()).magnitudeSquared() +
            ComponentWiseMax(vPos - vMax, Vector::zero()).magnitudeSquared() ) <= fTestRadius*fTestRadius;
}

#if defined(LEAP_SCENE_USE_UTIL_GL)
void SceneBox::DebugDrawGL( eStyle drawStyle ) const
{
  GLMatrixScope     matrixScope;
  const Vector      vSize   = GetSize() * GetScale();

  glMultMatrixf( GetTransform().toArray4x4() );

  glScalef( vSize.x, vSize.y, vSize.z );

  drawBox( drawStyle );

  if ( IsSelected() )
  {
    GLAttribScope attribScope( GL_CURRENT_BIT|GL_DEPTH_BUFFER_BIT );

    if ( drawStyle != kStyle_Outline )
    {
      glColor3f(1.0f, 1.0f, 1.0f);
      drawBox( kStyle_Outline );
    }

    glDisable(GL_DEPTH_TEST);
    const Vector vScale = LeapUtil::ComponentWiseReciprocal(vSize) * 0.125f;
    glScalef( vScale.x, vScale.y, vScale.z );

    drawAxes();
  }
}
#endif // LEAP_SCENE_USE_UTIL_GL

bool SceneCylinder::TestRayHit(const SceneRay& testRay, float& fHitDistOut) const
{
  SceneRay testRayObj = testRay.Transformed( GetWorldToObjectTransform() );

  const Vector& vAxis = Vector::yAxis();
  float radius        = m_fScale * m_fRadius;
  float height        = m_fScale * m_fHeight;
  Vector RxA          = testRayObj.m_vDirection.cross(vAxis);
  float norm          = RxA.magnitude();

  if (norm > 0)
  {
    Vector D = RxA/norm;
    float d = fabs(testRayObj.m_vOrigin.dot(D));
    if (d >= radius) {
      return false;
    }
    Vector O = D.cross(vAxis).normalized();
    float s = fabs(sqrt(radius*radius - d*d)/(testRayObj.m_vDirection.dot(O)));
    float temp = -((testRayObj.m_vOrigin).cross(vAxis)).dot(D)/norm;
    float tin = temp - s;
    float tout = temp + s;

    if (tin > 0 && fabs(testRayObj.CalcPointOn(tin).y) <= height/2.0)
    {
      fHitDistOut = tin;
      return true;
    }

    if (tout > 0 && fabs(testRayObj.CalcPointOn(tout).y) <= height/2.0)
    {
      fHitDistOut = tout;
      return true;
    }
  }
  return false;
}


bool SceneCylinder::TestSphereHit(const Vector& vTestPoint, float fTestRadius) const
{
  // by converting test point to object space the test is simplified.
  // we can check the test sphere against a cylinder centered at 0,0,0
  // with an axis along y+ (0,1,0)
  const Vector  vTestPosObj         = WorldToObjectPoint(vTestPoint);
  // scale-adjusted values of the cylinder
  const float   fCylRadius          = m_fRadius * m_fScale;
  const float   fHalfCylHeight      = m_fHeight * m_fScale * 0.5f;
  // vector from infinite line of cylinder axis to test point
  const Vector  vAxisToPoint        = Vector(vTestPosObj.x, 0, vTestPosObj.z);
  // square of distance from infinite line of cylinder axis to test point
  const float   fAxisToPointDistSq  = vAxisToPoint.magnitudeSquared();

  // only bother with the rest of the checks if the distance to the axis is less than
  // the sum of the cylinder radius and test sphere radius. any farther and they could not be touching.
  if (fAxisToPointDistSq < (fTestRadius + fCylRadius)*(fTestRadius + fCylRadius))
  {
    // distance from the vertical center of the cylinder to the test point projected onto the axis.
    const float fAbsDistAlongAxis = fabs(vTestPosObj.y);

    // simplest case - sphere vs. wall of cylinder.
    // the first check was whether or not the test sphere was within contact of the axis.
    // if the distance along the axis is inside the cylinder the sphere is in contact with the wall.
    if (fAbsDistAlongAxis < fHalfCylHeight)
    {
      return true;
    }

    // next simplest case - sphere vs. centers of cylinder end caps.
    // if the distance from the axis is within the cylinder
    // and the the test point along the axis is within the sphere radius of the end
    // it's touching.
    if (fAxisToPointDistSq < (fCylRadius*fCylRadius) && fAbsDistAlongAxis < (fHalfCylHeight + fTestRadius) )
    {
      return true;
    }

    // final case - sphere vs. edges (corners) of cylinder end caps

    // this is the closest point on the cylinder edge to the test point.
    const Vector  vClosest =  fCylRadius*vAxisToPoint.normalized() +
                              Vector( 0, fHalfCylHeight * ((vTestPosObj.y < 0.0f) ? -1.0f : 1.0f), 0 );

    // if the distance from the closest point to the test point is within the radius of
    // the test sphere it is in contact.
    if ((vTestPosObj-vClosest).magnitudeSquared() < fTestRadius*fTestRadius)
    {
      return true;
    }
  }

  return false;
}

#if defined(LEAP_SCENE_USE_UTIL_GL)
void SceneCylinder::DebugDrawGL( eStyle drawStyle ) const
{
  GLMatrixScope matrixScope;
  const float   fHeight = GetHeight() * GetScale();
  const float   fRadius = GetRadius() * GetScale();

  glMultMatrixf( GetTransform().toArray4x4() );

  glScalef( fRadius * 2, fHeight, fRadius * 2 );

  drawCylinder( drawStyle, kAxis_Y );

  if ( IsSelected() )
  {
    GLAttribScope attribScope( GL_CURRENT_BIT|GL_DEPTH_BUFFER_BIT );

    if ( drawStyle != kStyle_Outline )
    {
      glColor3f(1.0f, 1.0f, 1.0f);
      drawCylinder( kStyle_Outline, kAxis_Y );
    }

    glDisable(GL_DEPTH_TEST);
    const float fScaleXZ = 1.0f/(fRadius + fRadius) * 0.125f;
    glScalef( fScaleXZ, 1.0f/fHeight * 0.125f, fScaleXZ );
    drawAxes();
  }
}
#endif // LEAP_SCENE_USE_UTIL_GL

bool SceneDisk::TestRayHit(const SceneRay& testRay, float& fHitDistOut) const
{
  // by converting ray to object space we can do simpler testing.
  // in object space the normal is the z axis (0, 0, 1) and the
  // disk is a circle in the x/y plane centered at the origin (0, 0, 0)
  SceneRay  testRayObj   = testRay.Transformed( GetWorldToObjectTransform() );
  float     fRadius      = m_fScale * m_fRadius;
  // cosine of angle between test ray direction and surface normal of the disk
  //float     fCosRayAngle = testRayObj.m_vDirection.z;

  // if the z component of the ray direction is 0 the direction is all on the x/y plane
  // which means it is parallel to the disk and will not intersect.
  if (fabs(testRayObj.m_vDirection.z) < kfEpsilon)
  {
    return false;
  }

  // distance from the ray origin to the disk plane along the path of the ray
  float fPlaneHitDist = -testRayObj.m_vOrigin.z/testRayObj.m_vDirection.z;

  // if the plane hit distance is negative the ray is pointing away from the disk
  // if positive it is pointing towards the plane.
  if ( fPlaneHitDist > 0 )
  {
    // intersection point of the ray with the plane of the disk
    const Vector vPointOnPlane = testRayObj.CalcPointOn( fPlaneHitDist );

    // check if the point is within the disk - if so, it's a hit, otherwise not.
    if ( vPointOnPlane.magnitudeSquared() < fRadius * fRadius )
    {
      fHitDistOut = fPlaneHitDist;
      return true;
    }
  }

  return false;
}

bool SceneDisk::TestSphereHit(const Vector& vTestPoint, float fTestRadius) const
{
  // by converting the test point to object space we can do simpler testing.
  // in object space the normal is the z axis (0, 0, 1) and the
  // disk is a circle in the x/y plane centered at the origin (0, 0, 0)
  const Vector  vPos    = WorldToObjectPoint(vTestPoint);
  const float   fDiskRadius = (m_fScale*m_fRadius);

  // the test point has to be within the test sphere radius of the disk plane or it isn't touching.
  if (fabs(vPos.z) < fTestRadius)
  {
    // projection of the test point onto the plane of the disk.
    const Vector  vOnPlane( vPos.x, vPos.y, 0 );
    const float   fSin = vPos.z/fTestRadius;
    const float   fCos = sqrtf(1 - fSin*fSin);

    // fCos * fTestRadius is the radius of the circular intersection of the test sphere and the plane of the disk
    // add the two together and that's the maximum distance the projection of the test point onto the disk plane
    // can be from the center and still be in contact.
    fTestRadius =  fCos * fTestRadius + fDiskRadius;

    if (vOnPlane.magnitudeSquared() < fTestRadius * fTestRadius)
    {
      return true;
    }
  }

  return false;
}

#if defined(LEAP_SCENE_USE_UTIL_GL)
void SceneDisk::DebugDrawGL( eStyle drawStyle ) const
{
  GLMatrixScope   matrixScope;
  const float     fRadius = GetRadius() * GetScale();

  glMultMatrixf( GetTransform().toArray4x4() );

  glScalef( fRadius + fRadius, fRadius + fRadius, fRadius + fRadius );

  drawDisk( drawStyle, kPlane_XY );

  if ( IsSelected() )
  {
    GLAttribScope attribScope( GL_CURRENT_BIT|GL_DEPTH_BUFFER_BIT );

    if ( drawStyle != kStyle_Outline )
    {
      glColor3f(1.0f, 1.0f, 1.0f);
      drawDisk( kStyle_Outline, kPlane_XY );
    }

    glDisable(GL_DEPTH_TEST);
    const float fScale = 1.0f/(fRadius + fRadius) * 0.125f;
    glScalef( fScale, fScale, fScale );
    drawAxes();
  }
}
#endif // LEAP_SCENE_USE_UTIL_GL

bool ScenePlane::TestRayHit(const SceneRay& testRay, float& fHitDistOut) const
{
  const float fRayAngleCos = GetNormal().dot( testRay.m_vDirection );

  if (fabs(fRayAngleCos) < kfEpsilon)
  {
    return false;
  }

  // direct distance from the ray starting position to the plane
  const float fRayOriginPlaneDist = GetNormal().dot(GetCenter() - testRay.m_vOrigin);

  // distance from the ray origin point to the plane along the path of the ray
  const float fPlaneHitDist = fRayOriginPlaneDist/fRayAngleCos;

  if (fPlaneHitDist > 0)
  {
    fHitDistOut = fPlaneHitDist;
    return true;
  }

  return false;
}

bool ScenePlane::TestSphereHit(const Vector& vTestPoint, float fTestRadius) const
{
  return fabs((vTestPoint - GetCenter()).dot(GetNormal())) < fTestRadius;
}

#if defined(LEAP_SCENE_USE_UTIL_GL)
void ScenePlane::DebugDrawGL( eStyle drawStyle ) const
{
  (void)drawStyle;
}
#endif // LEAP_SCENE_USE_UTIL_GL

bool SceneSphere::TestRayHit(const SceneRay& testRay, float& fHitDistOut) const
{
  const float   fRadius = m_fScale * m_fRadius;
  const Vector  O       = testRay.m_vOrigin - GetCenter();
  const float   b       = O.dot(testRay.m_vDirection);
  const float   c       = O.dot(O) - fRadius*fRadius;
  const float   disc    = b*b - c;

  if (disc > 0)
  {
    float sdisc = sqrtf(disc);
    float root = (-b - sdisc);

    if (root > 0)
    {
      fHitDistOut = root;
      return true;
    }

    root = (-b + sdisc);

    if (root > 0)
    {
      fHitDistOut = root;
      return true;
    }
  }

  return false;
}

bool SceneSphere::TestSphereHit(const Vector& vTestPoint, float fTestRadius) const
{
  const float fMaxDist = (m_fScale*m_fRadius + fTestRadius);
  return (GetCenter() - vTestPoint).magnitudeSquared() < (fMaxDist * fMaxDist);
}

#if defined(LEAP_SCENE_USE_UTIL_GL)
void SceneSphere::DebugDrawGL( eStyle drawStyle ) const
{
  GLMatrixScope matrixScope;

  const float   fRadius  = GetRadius() * GetScale();

  glMultMatrixf( GetTransform().toArray4x4() );

  glScalef( fRadius+fRadius, fRadius+fRadius, fRadius+fRadius );

  drawSphere( drawStyle );

  if ( IsSelected() )
  {
    GLAttribScope attribScope( GL_CURRENT_BIT|GL_DEPTH_BUFFER_BIT );

    if ( drawStyle != kStyle_Outline )
    {
      glColor3f(1.0f, 1.0f, 1.0f);
      drawSphere( kStyle_Outline );
    }

    glDisable(GL_DEPTH_TEST);

    const float fScale = 1.0f/(fRadius + fRadius) * 0.125f;
    glScalef( fScale, fScale, fScale );

    drawAxes();
  }
}
#endif // LEAP_SCENE_USE_UTIL_GL

} // namespace Leap
