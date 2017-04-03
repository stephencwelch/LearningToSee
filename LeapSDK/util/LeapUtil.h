/******************************************************************************\
* Copyright (C) Leap Motion, Inc. 2011-2013.                                   *
* Leap Motion proprietary and  confidential.  Not for distribution.            *
* Use subject to the terms of the Leap Motion SDK Agreement available at       *
* https://developer.leapmotion.com/sdk_agreement, or another agreement between *
* Leap Motion and you, your company or other organization.                     *
\******************************************************************************/

#ifndef __LeapUtil_h__
#define __LeapUtil_h__

#include "Leap.h"

// Define integer types for Visual Studio 2005
#if defined(_MSC_VER) && (_MSC_VER < 1600)
typedef __int8 int8_t;
typedef __int16 int16_t;
typedef unsigned __int8 uint8_t;
typedef unsigned __int16 uint16_t;
#endif

namespace LeapUtil {

///
/// LeapMath is not intended to be a full-function vector math library.
/// It defines a few useful classes that are needed to pass vector values around and do a few
/// basic spatial operations.  You are highly encouraged to use a complete and optimized
/// vector math library for any serious computation.
///

///
/// math utility functions
/// To minimize additional code requirements for Leap samples no external math library is used.
/// These utility functions provide extra functionality not available in LeapMath.
/// In general this code is designed for transparency and simplicity rather than speed.
/// It is a minimal subset of extra functionality needed to implement LeapScene and
/// the sample code.  It is not intended to be a complete vector math utility library.
///

static const float kfPi       = Leap::PI;
static const float kf2Pi      = kfPi + kfPi;
static const float kfHalfPi   = kfPi * 0.5f;
static const float kfEpsilon  = 0.00001f;

template<typename T>
T Min( T lhs, T rhs )
{
  return lhs < rhs ? lhs : rhs;
}

template<typename T>
T Max( T lhs, T rhs )
{
  return lhs > rhs ? lhs : rhs;
}

template<typename T>
T Clamp( T inVal, T minVal, T maxVal )
{
  return (inVal < minVal) ? minVal : ((inVal > maxVal) ? maxVal : inVal);
}

/// works with Vector as well as floating point scalar types.
template<typename InterpType, typename ParamType>
InterpType Linterp( InterpType a, InterpType b, ParamType t )
{
  return a + static_cast<InterpType>((b - a) * t);
}

/// requires that the source vector type have direct member access with names x, y
template<class Vec2>
Leap::Vector FromVector2( const Vec2& vIn, float fZ = 0.0f )
{
  return Leap::Vector( static_cast<float>(vIn.x), static_cast<float>(vIn.y), fZ );
}

/// requires that the source vector type have direct member access with names x, y, z
template<class Vec3>
Leap::Vector FromVector3( const Vec3& vIn )
{
  return Leap::Vector( static_cast<float>(vIn.x), static_cast<float>(vIn.y), static_cast<float>(vIn.z) );
}

inline bool IsNearZero( float fVal )
{
 return fabs(fVal) <= kfEpsilon;
}

inline bool IsNearZero( const Leap::Vector& vVec )
{
  return IsNearZero(vVec.x) && IsNearZero(vVec.y) && IsNearZero(vVec.z);
}

/// works with Vectors as well as floats
template<typename T>
inline bool IsNearEqual( const T& a, const T& b )
{
  return IsNearZero( a - b );
}

/// create a new matrix with just the rotation block from the argument matrix
inline Leap::Matrix ExtractRotation( const Leap::Matrix& mtxTransform )
{
  return Leap::Matrix( mtxTransform.xBasis, mtxTransform.yBasis, mtxTransform.zBasis );
}

/// returns a matrix representing the inverse rotation by simple transposition of the rotation block.
inline Leap::Matrix RotationInverse( const Leap::Matrix& mtxRot )
{
  return Leap::Matrix(  Leap::Vector( mtxRot.xBasis.x, mtxRot.yBasis.x, mtxRot.zBasis.x ),
                        Leap::Vector( mtxRot.xBasis.y, mtxRot.yBasis.y, mtxRot.zBasis.y ),
                        Leap::Vector( mtxRot.xBasis.z, mtxRot.yBasis.z, mtxRot.zBasis.z ) );
}

/// returns a matrix that is the orthonormal inverse of the argument matrix.
/// this is only valid if the input matrix is orthonormal (the basis vectors are mutually perpendicular and of length 1)
inline Leap::Matrix RigidInverse( const Leap::Matrix& mtxTransform )
{
  Leap::Matrix  rigidInverse = RotationInverse( mtxTransform );
  rigidInverse.origin  = rigidInverse.transformDirection( -mtxTransform.origin );
  return rigidInverse;
}

inline Leap::Vector ComponentWiseMin( const Leap::Vector& vLHS, const Leap::Vector& vRHS )
{
  return Leap::Vector( Min( vLHS.x, vRHS.x ), Min( vLHS.y, vRHS.y ), Min( vLHS.z, vRHS.z ) );
}

inline Leap::Vector ComponentWiseMax( const Leap::Vector& vLHS, const Leap::Vector& vRHS )
{
  return Leap::Vector( Max( vLHS.x, vRHS.x ), Max( vLHS.y, vRHS.y ), Max( vLHS.z, vRHS.z ) );
}

inline Leap::Vector ComponentWiseScale( const Leap::Vector& vLHS, const Leap::Vector& vRHS )
{
  return Leap::Vector( vLHS.x * vRHS.x, vLHS.y * vRHS.y, vLHS.z * vRHS.z );
}

inline Leap::Vector ComponentWiseReciprocal( const Leap::Vector& vVec )
{
  return Leap::Vector( 1.0f/vVec.x, 1.0f/vVec.y, 1.0f/vVec.z );
}

inline float MinComponent( const Leap::Vector& vVec )
{
  return Min( vVec.x, Min( vVec.y, vVec.z ) );
}

inline float MaxComponent( const Leap::Vector& vVec )
{
  return Max( vVec.x, Max( vVec.y, vVec.z ) );
}

/// compute the polar/spherical heading of a vector direction in z/x plane
inline float Heading( const Leap::Vector& vVec )
{
  return atan2f( vVec.z, vVec.x );
}

/// compute the spherical elevation of a vector direction in y above the z/x plane
inline float Elevation( const Leap::Vector& vVec )
{
  return atan2f( vVec.y, sqrtf(vVec.z * vVec.z + vVec.x * vVec.x) );
}

/// convert from Cartesian (rectangular) coordinates to spherical coordinates
/// (magnitude, heading, elevation)
inline Leap::Vector CartesianToSpherical( const Leap::Vector& vCartesian )
{
  return Leap::Vector( vCartesian.magnitude(), Heading(vCartesian), Elevation(vCartesian) );
}

/// set magnitude to 1 and bring heading to (-Pi,Pi], elevation into [-Pi/2, Pi/2]
inline Leap::Vector NormalizeSpherical( const Leap::Vector& vSpherical )
{
  float fHeading  = vSpherical.y;
  float fElevation = vSpherical.z;

  for ( ; fElevation <= -kfPi; fElevation += kf2Pi );
  for ( ; fElevation > kfPi; fElevation -= kf2Pi );

  if ( fabs(fElevation) > kfHalfPi )
  {
    fHeading += kfPi;
    fElevation = fElevation > 0 ? (kfPi - fElevation) : -(kfPi + fElevation);
  }

  for ( ; fHeading <= -kfPi; fHeading += kf2Pi );
  for ( ; fHeading > kfPi; fHeading -= kf2Pi );

  return Leap::Vector( 1, fHeading, fElevation );
}

/// convert from spherical coordinates (magnitude, heading, elevation) to
/// Cartesian (rectangular) coordinates
inline Leap::Vector SphericalToCartesian( const Leap::Vector& vSpherical )
{
  const float fMagnitude    = vSpherical.x;
  const float fCosHeading   = cosf( vSpherical.y );
  const float fSinHeading   = sinf( vSpherical.y );
  const float fCosElevation = cosf( vSpherical.z );
  const float fSinElevation = sinf( vSpherical.z );

  return Leap::Vector(  fCosHeading   * fCosElevation  * fMagnitude,
                                        fSinElevation  * fMagnitude,
                        fSinHeading   * fCosElevation  * fMagnitude);
}

inline const char* BoolToStr( uint32_t bVal )
{
  return bVal ? "On" : "Off";
}

template<int _HistoryLength=256>
class RollingAverage
{
public:
  enum
  {
    kHistoryLength = _HistoryLength
  };

public:
  RollingAverage() { Reset(); }

  void Reset()
  {
    m_uiCurIndex  = 0;
    m_fNumSamples = 0.0f;
    m_fSum        = 0.0f;
    m_fAverage    = 0.0f;
    for ( int i = 0; i < kHistoryLength; m_afSamples[i++] = 0.0f );
  }

  float AddSample( float fSample )
  {
    m_fNumSamples =   Min( (m_fNumSamples + 1.0f), static_cast<float>(kHistoryLength) );
    m_fSum        -=  m_afSamples[m_uiCurIndex];
    m_fSum        +=  fSample;
    m_fAverage    =   m_fSum * (1.0f/m_fNumSamples);

    m_afSamples[m_uiCurIndex] = fSample;

    m_uiCurIndex = static_cast<uint32_t>((m_uiCurIndex + 1) % kHistoryLength);

    return m_fAverage;
  }

  float     GetAverage()    const { return m_fAverage; }
  float     GetSum()        const { return m_fSum; }
  uint32_t  GetNumSamples() const { return static_cast<uint32_t>(m_fNumSamples); }

  /// index 0 is the oldest sample, index kHistorySize - 1 is the newest.
  float     operator[](uint32_t uiIdx) const { return m_afSamples[(m_uiCurIndex + uiIdx) % kHistoryLength]; }

private:
  uint32_t      m_uiCurIndex;
  float         m_fNumSamples;
  float         m_fSum;
  float         m_fAverage;
  float         m_afSamples[kHistoryLength];
};

/// a graphics system agnostic camera that provides a point of view and a view matrix
/// as well as utility methods for moving the point of view around in useful ways.
/// field of view, aspect ratio and clipping planes are stored but not handled directly
/// by this class. no projection matrix created - the stored values can be used to
/// generate the proper projection matrix on whatever graphics system is used.
/// this class can also be sub-classed to suit a particular graphics system.
/// convention is right hand rule with z- forward, y+ up, x+ right
class Camera
{
public:
  Camera()
    : m_fOrbitDistance(-1.0f),
      m_fMaxOrbitLatitude( LeapUtil::kfPi * 0.375f ),
      m_fNearClip( 0.1f ),
      m_fFarClip( 128.0f ),
      m_fVerticalFOVDegrees( 40.0f ),
      m_fAspectRatio( 4.0f/3.0f )
  {}

  /// POV is the world space location and orientation of the camera.
  const Leap::Matrix& GetPOV() const { return m_mtxPOV; }

  void SetPOV( const Leap::Matrix& mtxPOV ) { m_mtxPOV = mtxPOV; }

  /// View is the world space to camera space transform.
  /// Useful for setting the view matrix in your rendering pipeline.
  Leap::Matrix GetView() const { return LeapUtil::RigidInverse(m_mtxPOV); }

  /// position of the camera in world space
  const Leap::Vector& GetPosition() const { return m_mtxPOV.origin; }

  void SetPosition( const Leap::Vector& vPos ) { m_mtxPOV.origin = vPos; }

  /// orientation of the camera in world space
  Leap::Matrix GetRotation() const { return LeapUtil::ExtractRotation(m_mtxPOV); }

  void SetRotation( const Leap::Vector& vAxis, float fRadians ) { m_mtxPOV.setRotation( vAxis, fRadians ); }

  void SetRotation( const Leap::Matrix& mtxRot )
  {
    m_mtxPOV.xBasis = mtxRot.xBasis;
    m_mtxPOV.yBasis = mtxRot.yBasis;
    m_mtxPOV.zBasis = mtxRot.zBasis;
  }

  /// places the camera a the given position with it pointing towards the specified target position
  void SetPOVLookAt( const Leap::Vector& vCameraPosition, const Leap::Vector& vTarget, Leap::Vector vUp = Leap::Vector::yAxis() );

  /// maintains the current camera position and turns it to look at the specified target position
  void SetRotationLookAt( const Leap::Vector& vTarget, const Leap::Vector& vUp = Leap::Vector::yAxis() )
  {
    SetPOVLookAt( m_mtxPOV.origin, vTarget, vUp );
  }

  /// translate the position of the camera (move it from its current position) by the specified offset
  void Move( const Leap::Vector& vDeltaPos )
  {
    m_mtxPOV.origin += vDeltaPos;
  }

  /// apply the specified rotation to the camera on top of its current rotation
  void Rotate( const Leap::Matrix& mtxRotation )
  {
    m_mtxPOV = m_mtxPOV * LeapUtil::ExtractRotation(mtxRotation);
  }

  void Rotate( const Leap::Vector& vAxis, float fRadians )
  {
    m_mtxPOV = m_mtxPOV * Leap::Matrix( vAxis, fRadians );
  }

  /// the point of interest for the camera to orbit around
  const Leap::Vector& GetOrbitTarget() const
  {
    return m_vOrbitTarget;
  }

  void SetOrbitTarget( const Leap::Vector& vOrbitTarget)
  {
    m_vOrbitTarget = vOrbitTarget;
    updateOrbitDistance();
  }

  /// rotate the camera about its orbit target.
  /// the orbit position is on a sphere.
  /// fDeltaMagnitude changes the distance of the camera to the target
  /// fDeltaLongitude is in radians and changes the position of the camera around the left-right direction on the sphere
  /// fDeltaLatitude is in radians and changes the position of the camera around the up-down direction of the sphere
  void RotateOrbit( float fDeltaMagnitude, float fDeltaLongitude, float fDeltaLatitude );

  /// the maximum vertical angle in radians that the camera can ascend/descend to during orbit movement.
  /// default is 3/4 of a right angle.
  float GetMaxOrbitLatitude() const
  {
    return m_fMaxOrbitLatitude;
  }

  void SetMaxOrbitLatitude( float fMaxOrbitLatitude )
  {
    m_fMaxOrbitLatitude = fMaxOrbitLatitude;
  }

  /// returns how far a world space point is down the line of sight of the camera.
  /// negative values are behind the camera.
  float CalcViewDepth( const Leap::Vector& vPos ) const
  {
    return m_mtxPOV.zBasis.dot( m_mtxPOV.origin - vPos );
  }

  /// convenience methods for mapping mouse movements to orbit camera movements.

  void OnMouseWheel( float fDeltaZ );

  void OnMouseDown( const Leap::Vector& vMousePos );

  void OnMouseMoveOrbit( const Leap::Vector& vMousePos );

  float GetNearClip() const { return m_fNearClip; }

  float GetFarClip() const { return m_fFarClip; }

  void SetNearClip( float fNearClip ) { m_fNearClip = Max( 0.01f, fNearClip ); }

  void SetFarClip( float fFarClip ) { m_fFarClip = fFarClip; }

  void SetClipPlanes( float fNearClip, float fFarClip )
  {
    SetNearClip( fNearClip );
    SetFarClip( fFarClip );
  }

  float GetVerticalFOVDegrees() const { return m_fVerticalFOVDegrees; }

  void SetVerticalFOVDegrees( float fFOV ) { m_fVerticalFOVDegrees = fFOV; }

  float GetAspectRatio() const { return m_fAspectRatio; }

  void SetAspectRatio( float fAspectRatio ) { m_fAspectRatio = fAspectRatio; }

private:
  void updateOrbitDistance()
  {
    float fOrbitDistance = (m_vOrbitTarget - m_mtxPOV.origin).magnitude();

    if ( !IsNearZero( fOrbitDistance - m_fOrbitDistance ) )
    {
      m_fOrbitDistance  = fOrbitDistance;
    }
  }

private:
  Leap::Matrix  m_mtxPOV;
  Leap::Vector  m_vLastMousePos;
  Leap::Vector  m_vOrbitTarget;
  float         m_fOrbitDistance;
  float         m_fMaxOrbitLatitude;
  float         m_fNearClip;
  float         m_fFarClip;
  float         m_fVerticalFOVDegrees;
  float         m_fAspectRatio;
};


/// Utility class for adding simple momentum to 2D or 3D UI elements.
class ScrollMomentum {
public:
  ScrollMomentum()
    : m_vDirection(Leap::Vector::yAxis()),
      m_fScrollSize(512.0f),
      m_fSpeed(0),
      m_fMinSpeed(0.125f),
      m_fDrag(0.4f),
      m_fDragPower(2.0f),
      m_fFixedTimeStep(1.0f/60.0f),
      m_fPendingDeltaTime(0.0f)
  {}

  /// The current scroll position.
  /// Calling SetPosition assigns an initial position value.
  /// Calls to update will change position based on velocity (speed * direction)
  /// The default value is the zero vector (0, 0, 0)
  const Leap::Vector& getPosition() const                             { return m_vPosition; }
  void                setPosition( const Leap::Vector& vPosition )    { m_vPosition = vPosition; }

  /// The direction of movement.
  /// Direction should never be set to the zero vector.
  /// The default value is Y+ vector (0, 1, 0)
  const Leap::Vector& getDirection() const                            { return m_vDirection; }
  void                setDirection( const Leap::Vector& vDirection )  { m_vDirection = vDirection.normalized(); }

  /// ScrollSize is used to tune the rate at which drag
  /// starts to taper its effect on speed.
  /// Because drag is proportional to speed^dragPower it slows
  /// objects moving at speeds greater than 1 quickly and tapers
  /// for speeds below 1.  The drag calculations are done
  /// using speed/ScrollSize as the value to square.
  /// ScrollSize is always > 0.
  /// The default value is 512
  float               getScrollSize() const                                 { return m_fScrollSize; }
  void                setScrollSize( float fScrollSize )                    { m_fScrollSize = (fScrollSize > 0.0f ? fScrollSize : 1.0f); }

  /// Current speed of movement.
  /// Speed is not defined in strict terms of any particular unit.
  /// A value in pixels/second or mm/sec will work well for speed.
  /// Calling SetSpeed assigns an initial speed value.
  /// Calls to update will change speed as drag slows down movement.
  /// Speed can be positive or negative.
  float               getSpeed() const                                      { return m_fSpeed; }
  void                setSpeed( float fSpeed )                              { m_fSpeed = fSpeed; }

  /// When absolute value of speed falls to or below MinSpeed speed is set to 0.
  /// This is to prevent asymptotically approaching the limit of floating
  /// point resolution and getting jitter or undesirable effects.
  /// The default value is 0.125
  float               getMinSpeed() const                                   { return m_fMinSpeed; }
  void                setMinSpeed( float fMinSpeed )                        { m_fMinSpeed = fabs(fMinSpeed); }

  /// A fixed time step is used to ensure consistent behavior.  The deltaTime passed to
  /// update is processed in increments of the fixed time step.  Any unhandled time is
  /// cached for processing on the next call to update.
  /// The default value is 1/60th of a second.
  float               getFixedTimeStep() const                              { return m_fMinSpeed; }
  void                setFixedTimeStep( float fFixedTimeStep )              { m_fFixedTimeStep = fabs(fFixedTimeStep); }

  /// The coefficient of drag to use for slowing movement over time.
  /// Drag is always a positive value.
  /// The default value is 0.4
  float               getDrag() const                                       { return m_fDrag; }
  void                setDrag( float fDrag )                                { m_fDrag = fabs(fDrag); }

  /// Drag power controls how drag is calculated from speed.
  /// Drag force is (speed/scrollSize)^dragPower
  /// A drag power of 1 would be linear falloff.
  /// The default value is 2.0 (square falloff)
  float               getDragPower() const                                  { return m_fDragPower; }
  void                setDragPower( float fDragPower )                      { m_fDragPower = fDragPower; }

  /// Current velocity (scaled direction vector) - changes over time as drag slows movement.
  /// Convenience method - just multiplies direction by current speed.
  const Leap::Vector  getVelocity() const                             { return m_vDirection * m_fSpeed; }

  /// Convenience method for setting a direction and initial speed in one call.
  /// see SetDirection() and SetSpeed()
  void setVelocity( const Leap::Vector& vDirection, float fSpeed )
  {
    setDirection( vDirection );
    setSpeed( fSpeed );
  }

  /// Update handles time stepping.
  /// Update modifies velocity by drag and position by velocity.
  /// Time is processed fixed size chunks.  If _deltaTimeSeconds is larger than that
  /// chunk, update will iterate through the time in increments of the fixed time step.
  /// If it is smaller, the delta time is accumulated over successive calls to update
  /// until one time step has passed and then it is processed.
  void  update( float fDeltaTimeSeconds )
  {
    if ( fDeltaTimeSeconds <= 0.0f )
    {
      return;
    }

    if ( fabs(m_fSpeed) > m_fMinSpeed )
    {
      if ( m_fDrag > 0 )
      {
        // only force to fixed time step if doing drag calculation

        const float fMinScrollSpeed = m_fMinSpeed / m_fScrollSize;
        // fold the sign of speed into the scaling factor
        const float fScrollScale    = (m_fSpeed < 0.0f ? -1.0f : 1.0f) * m_fScrollSize;
        // absolute value of scaled speed (_scrollScale has same sign as speed, guaranteeing positive value)
        float fScrollSpeed          = m_fSpeed / fScrollScale;

        // include any left over time from previous update in the time to process for this update.
        fDeltaTimeSeconds += m_fPendingDeltaTime;

        // process time since last update in fixed chunks
        for ( ; fDeltaTimeSeconds >= m_fFixedTimeStep; fDeltaTimeSeconds -= m_fFixedTimeStep )
        {
          const float fDragForce = powf( fScrollSpeed, m_fDragPower ) * m_fDrag;

          // decelerate unsigned/scaled speed by drag
          fScrollSpeed -= fDragForce * m_fFixedTimeStep;

          // bail out if speed has hit minimum threshold
          if ( fScrollSpeed <= fMinScrollSpeed )
          {
            m_fSpeed = 0.0f;
            fDeltaTimeSeconds = 0.0f;
            break;
          }

          // update signed/unscaled speed
          m_fSpeed = fScrollSpeed * fScrollScale;

          // step position by velocity
          m_vPosition += m_vDirection * (m_fFixedTimeStep * m_fSpeed);
        }

        // keep track of any leftover time not processed during this update.
        m_fPendingDeltaTime = fDeltaTimeSeconds;
      }
      else
      {
        // simple position change with velocity and no drag - not affected by large time steps. no looping needed.
        m_vPosition += m_vDirection * ((m_fPendingDeltaTime + fDeltaTimeSeconds) * m_fSpeed);
        m_fPendingDeltaTime = 0.0f;
      }
    }
    else
    {
      // enforce zeroing of speeds below minimum absolute value.
      m_fSpeed = 0.0f;
      m_fPendingDeltaTime = 0.0f;
    }
  }

protected:
  Leap::Vector  m_vPosition;
  Leap::Vector  m_vDirection;
  float         m_fScrollSize;
  float         m_fSpeed;
  float         m_fMinSpeed;
  float         m_fDrag;
  float         m_fDragPower;
  float         m_fFixedTimeStep;
  float         m_fPendingDeltaTime;
};


/// default destruction template class used by smart pointer.
template<typename T>
class SmartInstanceDestructor
{
public:
  static void Destroy( T* pPointer ) { delete pPointer; }
};

/// alternative destructor for arrays of objects
template<typename T>
class SmartArrayDestructor
{
public:
  static void Destroy( T* pPointer ) { delete[] pPointer; }
};

/// smart pointer template class.
/// maintains reference count that is incremented for each
/// new instance of a smart pointer that refers to the same raw pointer
/// and decremented each time a reference is released.
/// a reference is released when
/// a smart pointer goes out of scope
/// OR has another value assigned to it
/// OR .Release() is called explicitly.
/// when the reference count reaches zero the Destructor::Destroy static method
/// is invoked on the raw pointer.
/// note: this smart pointer class is not thread-safe.
/// making it thread safe would have required adding a cross-platform
/// mutex class.  the use cases in the sample code do not require it.
template< typename T,
          class Destructor = SmartInstanceDestructor<T>,
          unsigned int ManagedPointerPoolSize = 512 >
class SmartPointer
{
public:
  enum { kManagedPointerPoolSize = ManagedPointerPoolSize };

  typedef T ManagedType;

private:
  // a managed pointer is a pointer of the desired type plus a reference count.
  struct ManagedPointerEntry
  {
    ManagedType*  m_pPointer;
    uint32_t      m_uiRefCount;
  };

  // this private, embedded class manages a pool of smart pointers for the given type.
  // there are separate pools for each created type of smart pointer.
  // the pool is managed using a simple O(1) fixed size free list allocator
  struct ManagedPointerPool
  {
    enum { kPoolSize = kManagedPointerPoolSize, kDeadPointer = 0xdeadbea7 };

    ManagedPointerPool()
      : m_uiNextFree(0),
        m_uiNumAllocated(0)
    {
      for ( uint32_t i = 0; i < static_cast<uint32_t>(kPoolSize); i++ )
      {
        m_aEntryPool[i].m_pPointer    = reinterpret_cast<ManagedType*>(kDeadPointer);
        m_aEntryPool[i].m_uiRefCount  = i + 1;
      }
    }

    // allocate a managed pointer entry for a raw pointer
    ManagedPointerEntry* allocEntry( ManagedType* pPointer )
    {
      ManagedPointerEntry* pEntry = NULL;

      // this incurs a linear search penalty each time
      // we create a new smart pointer from a raw pointer
      // but adds significant safety - it prevents
      // assigning the same raw pointer to two different
      // member entries - this would result in double
      // deletion when the 2nd one hit its end of life.
      pEntry = findEntry( pPointer );

      if ( pEntry )
      {
        // we found an existing managed pointer entry, just increase the reference count
        pEntry->m_uiRefCount++;
      }
      else if ( (m_uiNextFree < kPoolSize) && pPointer )
      {
        // grab the next free entry from the head of the free list
        pEntry = m_aEntryPool + m_uiNextFree;

        // the head of the list now becomes the next free entry in the chain
        // (m_uiRefCount is serving double-duty as next index member for entries that are still in the pool)
        m_uiNextFree = pEntry->m_uiRefCount;

        // initialize the entry with the pointer and a reference count of 1.
        pEntry->m_pPointer = pPointer;
        pEntry->m_uiRefCount = 1;

        // book keeping.
        m_uiNumAllocated++;
      }

      return pEntry;
    }

    // return an entry to the pool
    void freeEntry( ManagedPointerEntry* pEntry )
    {
      // calculate the entry index as an offset from the start of the pool
      const uint32_t entryIndex = static_cast<uint32_t>(pEntry - m_aEntryPool);

      // if this is a valid allocated entry (from this pool, and not already freed)
      if (  (entryIndex < kPoolSize)  &&
            (m_uiNumAllocated != 0)   &&
            (pEntry->m_pPointer != reinterpret_cast<ManagedType*>(kDeadPointer)) )
      {
        // clear the entry
        pEntry->m_pPointer = NULL;

        // push it to the head of the free list
        pEntry->m_uiRefCount = m_uiNextFree;
        m_uiNextFree = entryIndex;

        // book keeping.
        m_uiNumAllocated--;
      }
    }

    // look for the managed pointer entry associated with a raw pointer. returns NULL
    // if pPointer is NULL, dead or not found.
    ManagedPointerEntry* findEntry( const ManagedType* pPointer )
    {
      // don't bother if the pointer is NULL or marked as our dead value
      if ( pPointer && (pPointer != reinterpret_cast<ManagedType*>(kDeadPointer)) )
      {
        // linear search. stop after testing as many live entries as are allocated instead of always
        // searching the entire pool.
        for ( uint32_t i = 0, j = 0; j < m_uiNumAllocated && i < static_cast<uint32_t>(kPoolSize); i++ )
        {
          // if the entry is live (allocated)
          if ( m_aEntryPool[i].m_pPointer != reinterpret_cast<ManagedType*>(kDeadPointer) )
          {
            // a match? return it.
            if ( m_aEntryPool[i].m_pPointer == pPointer )
            {
              return m_aEntryPool + i;
            }

            // not, increment the count of live entries encountered.
            j++;
          }
        }
      }

      return NULL;
    }

    // number of allocated managed pointer entries
    uint32_t getNumAllocated() const { return m_uiNumAllocated; }

    // number of available managed pointer entries.
    uint32_t getNumFree() const { return static_cast<uint32_t>(kPoolSize) - m_uiNumAllocated; }

    // actual entry pool
    ManagedPointerEntry m_aEntryPool[kPoolSize];

    // head of the free list
    uint32_t            m_uiNextFree;

    // number of entries that have been allocated from the free list
    uint32_t            m_uiNumAllocated;
  };

  // static instance of the managed pointer pool
  static ManagedPointerPool& s_pool()
  {
    static ManagedPointerPool _s_pool;
    return _s_pool;
  }

  /// increment the reference count on our shared managed pointer entry (if any)
  void refInc()
  {
    if ( m_pManagedPointer )
    {
      m_pManagedPointer->m_uiRefCount++;
    }
  }

  /// decrement the reference count on our shared managed pointer entry (if any)
  /// if the reference count hits 0 the raw pointer is passed to Destructor::Destroy
  /// and the managed pointer entry is returned to the pool.
  void refDec()
  {
    if ( m_pManagedPointer )
    {
      if ( m_pManagedPointer->m_uiRefCount )
      {
        m_pManagedPointer->m_uiRefCount--;
      }

      if ( !m_pManagedPointer->m_uiRefCount )
      {
        Destructor::Destroy(m_pManagedPointer->m_pPointer);
        s_pool().freeEntry(m_pManagedPointer);
      }

      m_pManagedPointer = NULL;
    }
  }

public:
  /// default constructor - equivalent of a NULL pointer.
  SmartPointer()
    : m_pManagedPointer(NULL)
  {
  }

  /// construction of a smart pointer from a raw pointer is not implicit - they should be created once at the beginning
  /// of an object's life and passed around/dereferenced.  creating a new smart pointer from a raw pointer
  /// should not happen unintentionally.
  /// assignment operator for raw pointer type does not exist for the same reason that the raw pointer
  /// constructor is explicit.  creation of a smart pointer from a raw pointer should be deliberate.
  explicit SmartPointer( ManagedType* pManaged )
    : m_pManagedPointer( s_pool().allocEntry( pManaged ) )
  {
  }

  /// copy constructor. increases reference count for shared managed pointer entry
  SmartPointer( const SmartPointer& rhs )
    : m_pManagedPointer( rhs.m_pManagedPointer )
  {
    refInc();
  }

  /// assignment operator overload for smart pointer to smart pointer.
  /// releases old managed entry and increases reference count for new one.
  const SmartPointer& operator =( const SmartPointer& rhs )
  {
    if ( m_pManagedPointer != rhs.m_pManagedPointer )
    {
      refDec();
      m_pManagedPointer = rhs.m_pManagedPointer;
      refInc();
    }

    return *this;
  }

  /// destructor decreases reference count for managed pointer entry.
  ~SmartPointer()
  {
    refDec();
  }

  /// explicit access to the managed pointer. usually the overloaded cast and -> operators are
  /// sufficient but it is sometimes useful to have a named method to call.
  ManagedType* GetPointer() const { return m_pManagedPointer ? m_pManagedPointer->m_pPointer : NULL; }

  /// how many references are there to this managed pointer?
  uint32_t GetRefCount() const { return m_pManagedPointer ? m_pManagedPointer->m_uiRefCount : 0u; }

  /// operators for easy implicit assignment to raw pointer type or direct use of the object pointer
  operator ManagedType*() const { return GetPointer(); }

  ManagedType* operator ->() const { return GetPointer(); }

  /// boolean and comparison operator overloads
  operator bool() const { return m_pManagedPointer != NULL; }

  bool operator !() const { return !m_pManagedPointer; }

  bool operator ==( const ManagedType* pPointer ) const { return GetPointer() == pPointer; }

  bool operator !=( const ManagedType* pPointer ) const { return GetPointer() != pPointer; }

  bool operator ==( const SmartPointer& rhs ) const { return m_pManagedPointer == rhs.m_pManagedPointer; }

  bool operator !=( const SmartPointer& rhs ) const { return m_pManagedPointer != rhs.m_pManagedPointer; }

  /// calling Release is an alternative to assigning SmartPointer::Null().
  /// refDec() is called in the destructor of the smart pointer - it is not necessary
  /// to call Release() in the destructor of a class containing a smart pointer.
  void Release()
  {
    refDec();
  }

  /// convenient static method for returning in cases where a null return value is needed.
  static const SmartPointer& Null()
  {
    static SmartPointer _s_null;
    return _s_null;
  }

  /// returns true if the given raw pointer is managed by a smart pointer somewhere, false if not.
  static bool IsManaged( const ManagedType* pPointer )
  {
    return s_pool().findEntry( pPointer ) != NULL;
  }

  /// returns the number of raw pointers of this type under management.
  /// use SmartPointer::kManagedPointerPoolSize to get the total number of pointers of this type that can be managed.
  static uint32_t GetNumManagedPointers() { return s_pool().getNumAllocated(); }

private:
  ManagedPointerEntry* m_pManagedPointer;
};

} // namespace Leap

#endif // __LeapUtil_h__
