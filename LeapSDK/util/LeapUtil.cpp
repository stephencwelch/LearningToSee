/******************************************************************************\
* Copyright (C) Leap Motion, Inc. 2011-2013.                                   *
* Leap Motion proprietary and  confidential.  Not for distribution.            *
* Use subject to the terms of the Leap Motion SDK Agreement available at       *
* https://developer.leapmotion.com/sdk_agreement, or another agreement between *
* Leap Motion and you, your company or other organization.                     *
\******************************************************************************/
#include "LeapUtil.h"

namespace LeapUtil {

using namespace Leap;

///
/// Camera methods
///

void Camera::SetPOVLookAt( const Vector& vCameraPosition, const Vector& vTarget, Vector vUp )
{
  Vector vZBasis = (vCameraPosition - vTarget).normalized();

  // if the zero vector was passed try to preserve the current camera up as much as possible
  if ( vUp == Vector::zero() )
  {
    vUp = m_mtxPOV.yBasis;
  }
  else
  {
    vUp = vUp.normalized();
  }

  // if the z direction is parallel to the desired up direction
  // we need to pick another up.
  if ( IsNearZero(1.0f - fabs(vZBasis.dot(vUp))) )
  {
    // make the new up perpendicular to the plane of the new z axis and the old right direction.
    // it's a decent guess.
    vUp = vZBasis.cross( m_mtxPOV.xBasis ).normalized();
  }

  Vector vXBasis = vUp.cross(vZBasis).normalized();
  Vector vYBasis = vZBasis.cross( vXBasis ).normalized();

  m_mtxPOV.xBasis = vXBasis;
  m_mtxPOV.yBasis = vYBasis;
  m_mtxPOV.zBasis = vZBasis;
  m_mtxPOV.origin = vCameraPosition;

  updateOrbitDistance();
}

void Camera::RotateOrbit( float fDeltaMagnitude, float fDeltaLongitude, float fDeltaLatitude )
{
  m_fOrbitDistance += fDeltaMagnitude;

  Vector  vSphereZBasis = CartesianToSpherical( m_mtxPOV.zBasis );
  float   fNewLatitude  = Clamp(  vSphereZBasis.z + fDeltaLatitude,
                                  -m_fMaxOrbitLatitude,
                                  m_fMaxOrbitLatitude );

  Vector  vSphereNewZBasis( 1, vSphereZBasis.y + fDeltaLongitude, fNewLatitude );
  Vector  vSphereNewYBasis( 1, vSphereNewZBasis.y, vSphereNewZBasis.z + kfHalfPi );

  m_mtxPOV.zBasis = SphericalToCartesian( NormalizeSpherical( vSphereNewZBasis ) );
  m_mtxPOV.yBasis = SphericalToCartesian( NormalizeSpherical( vSphereNewYBasis ) );
  m_mtxPOV.xBasis = m_mtxPOV.yBasis.cross( m_mtxPOV.zBasis ).normalized();
  m_mtxPOV.yBasis = m_mtxPOV.zBasis.cross( m_mtxPOV.xBasis ).normalized();
  m_mtxPOV.origin = m_mtxPOV.zBasis * m_fOrbitDistance;
}

void Camera::OnMouseWheel( float fDeltaZ )
{
  static const float kfZScale = -0.15f;

  Move( m_mtxPOV.zBasis * fDeltaZ * kfZScale * m_fOrbitDistance );
  updateOrbitDistance();
}

void Camera::OnMouseDown( const Vector& vMousePos )
{
  m_vLastMousePos   = vMousePos;
  updateOrbitDistance();
}

void Camera::OnMouseMoveOrbit( const Vector& vMousePos )
{
  static const float kfMouseOrbitAngleScale = 1/1024.0f;

  Vector vOrbitDelta  = (vMousePos - m_vLastMousePos) * kfMouseOrbitAngleScale * kf2Pi;

  RotateOrbit( 0, vOrbitDelta.x, vOrbitDelta.y );

  m_vLastMousePos = vMousePos;
}

}
