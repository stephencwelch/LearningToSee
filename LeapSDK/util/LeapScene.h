/******************************************************************************\
* Copyright (C) Leap Motion, Inc. 2011-2013.                                   *
* Leap Motion proprietary and  confidential.  Not for distribution.            *
* Use subject to the terms of the Leap Motion SDK Agreement available at       *
* https://developer.leapmotion.com/sdk_agreement, or another agreement between *
* Leap Motion and you, your company or other organization.                     *
\******************************************************************************/

#if !defined(__LeapScene_h__)
#define __LeapScene_h__

#include "Leap.h"
#include "LeapUtil.h"

#if defined(LEAP_SCENE_USE_UTIL_GL)
  #include "LeapUtilGL.h"
#endif

// define this macro if you want SceneObject::GetAs<> to use
// simple internally implemented typeid checks
// instead of dynamic casting.
// some code bases prohibit the use of dynamic_cast.
// this provides a basic alternative.
// see the comment at SceneObject::GetAs<> below
#undef LEAP_SCENE_NO_DYNAMIC_CAST

#undef LEAP_EXPORT
#define LEAP_EXPORT
#undef LEAP_EXPORT_CLASS
#define LEAP_EXPORT_CLASS
#undef LEAP_EXPORT_PLUGIN
#define LEAP_EXPORT_PLUGIN

namespace Leap {

/// the scene class owns and manages the life cycle of scene objects.
/// it also handles updating and hit testing.
class Scene;

/// base scene object class - has 2 pure virtual methods TestSphereHit and TestRayHit
class SceneObject;

/// smart pointer for scene objects.  when caching pointers to scene objects
/// don't cache raw pointers or the object may be deleted leaving the pointer invalid.
/// the smart pointers have reference counts that will prevent the object from getting deleted until
/// all references are released.  references are released when the smart pointer goes out of scope
/// or has SceneObjectPtr::Null() assigned to it.
/// if an object is removed from the scene the pointer will remain valid
/// but the object will be an orphan - GetScene() will return NULL.
typedef LeapUtil::SmartPointer<SceneObject> SceneObjectPtr;

/// ray used for ray hit tests
struct SceneRay
{
  SceneRay() {}

  SceneRay( const Vector& vOrigin, const Vector& vDirection ) : m_vOrigin( vOrigin ), m_vDirection( vDirection ) {}

  /// returns a point that is the specified distance down the ray.
  Vector CalcPointOn( float fDistFromOrigin ) const { return m_vOrigin + m_vDirection * fDistFromOrigin; }

  /// change the orientation and position of this ray by the given transform
  void Transform( const Matrix& mtxTransform )
  {
    m_vOrigin = mtxTransform.transformPoint( m_vOrigin );
    m_vDirection = mtxTransform.transformDirection( m_vDirection );
  }

  SceneRay Transformed( const Matrix& mtxTransform ) const
  {
    return SceneRay(  mtxTransform.transformPoint( m_vOrigin ),
                      mtxTransform.transformDirection( m_vDirection ) );
  }

  Vector  m_vOrigin;
  Vector  m_vDirection;
};

/// stores the result of a ray test that hit something
struct SceneRayHit
{
#if defined(LEAP_SCENE_USE_UTIL_GL)
  void DebugDrawGL( float fHitSphereSize ) const;
#endif

  Vector          m_hitPoint;
  SceneRay        m_ray;
  int             m_iPointableID;
  float           m_fHitDistance;
  SceneObjectPtr  m_pHitObject;
};

/// types of interactions that may be performed on a scene object
enum eInteractionType
{
  kIT_Rotation          = 1 << 0,
  kIT_Translation       = 1 << 1,
  kIT_Scale             = 1 << 2,
  kIT_SelectionChange   = 1 << 3,
  kIT_IsSelected        = 1 << 4
};

/// the details of a potential scene interaction.
/// the potential interactions are queued up by the Scene each update and the
/// API consumer can decide which ones to accept or filter out.
class SceneInteraction
{
  friend class Scene;

protected:
  SceneInteraction() : m_fScale(1), m_uiFlags(0) {}

public:
  /// convenient predicates for represented interaction types
  bool HasSelectionChange()   const { return (m_uiFlags & kIT_SelectionChange) != 0; }
  bool HasRotation()          const { return (m_uiFlags & kIT_Rotation) != 0; }
  bool HasTranslation()       const { return (m_uiFlags & kIT_Translation) != 0; }
  bool HasScale()             const { return (m_uiFlags & kIT_Scale) != 0; }

  bool HasInteraction( eInteractionType interactionType ) const
  {
    return ((m_uiFlags >> static_cast<uint32_t>(interactionType)) & 1) != 0;
  }

  void ClearSelectionChange() { m_uiFlags &= ~kIT_SelectionChange; }
  void ClearRotation()        { m_uiFlags &= ~kIT_Rotation; }
  void ClearTranslation()     { m_uiFlags &= ~kIT_Translation; }
  void ClearScale()           { m_uiFlags &= ~kIT_Scale; }

  void ClearInteraction( eInteractionType interactionType )
  {
    m_uiFlags &= ~(1 << static_cast<uint32_t>(interactionType));
  }

  bool HasAnyInteraction() const
  {
    static const uint32_t kInteractionMask =  kIT_SelectionChange  |
                                              kIT_Rotation         |
                                              kIT_Translation      |
                                              kIT_Scale;
    return (m_uiFlags & kInteractionMask) != 0;
  }

  /// accessors to interaction values - validity depends on what types of interactions are in the flags.
  bool                  IsSelected()      const { return (m_uiFlags & kIT_IsSelected) != 0; }
  Matrix                GetRotation()     const { return LeapUtil::ExtractRotation( m_mtxTransform ); }
  float                 GetScale()        const { return m_fScale; }
  Vector                GetTranslation()  const { return m_mtxTransform.origin; }
  Matrix                GetTransform()    const { return m_mtxTransform; }

  /// scene object this interaction is associated with
  const SceneObjectPtr& GetObject()       const { return m_pObject; }

private:
  Matrix          m_mtxTransform;
  SceneObjectPtr  m_pObject;
  float           m_fScale;
  uint32_t        m_uiFlags;
};

/// contact point between a pointable (finger or tool) and a scene object.
struct SceneContactPoint
{
  SceneContactPoint() : m_iPointableID(-1) {}
  SceneContactPoint( const Vector& vPoint, int iPointableID ) : m_vPoint( vPoint ), m_iPointableID( iPointableID ) {}
  Vector  m_vPoint;
  int     m_iPointableID;
};

/// scene manages scene objects - handles selection and movement
class LEAP_EXPORT_CLASS Scene
{
public:
  enum eFlag
  {
    kF_UpdateRayCast = 1 << 0,
    kF_UpdateContact = 1 << 1
  };

  enum
  {
    kMaxObjects             = 512,
    kMaxRayHits             = 32,
    kInteractionQueueLength = 32
  };

  LEAP_EXPORT Scene();

  LEAP_EXPORT virtual ~Scene()
  {
    Reset();
  }

  /// AddObject allocates new objects and add them to the list of scene children to manage.
  /// raw pointers are returned for convenience.  use SceneObjectPtr type when storing scene object
  /// references for later use.
  /// you can not add an existing instance of an externally allocated object for management.
  /// the scene always creates the managed instance.
  /// this restriction allows the object management to stay simple and concise.
  template<class ObjectClass>
  ObjectClass* AddObject()
  {
    return allocateObject<ObjectClass>();
  }

  /// RemoveObject() marks an object for removal.  It is not actually removed until the next call to Update().
  /// Removal does not delete the object, it orphans the object (unsets the scene member) and
  /// releases the scene's reference.  When the last smart pointer reference to the object is released
  /// it will be deleted.
  /// objects already marked for removal or not belonging to this scene are ignored.
  LEAP_EXPORT void RemoveObject( SceneObject* pObject );

  /// Reset() removes all objects directly.  does not queue for later processing.
  /// as with RemoveObject() the objects are not deleted, only orphaned and released.
  /// When the last smart pointer reference to the objects are released they are deleted.
  LEAP_EXPORT void Reset();

#if defined(LEAP_SCENE_USE_UTIL_GL)
  LEAP_EXPORT void RayHitsDebugDrawGL() const;
#endif

  const SceneObjectPtr& GetObjectByIndex( int idx ) const
  {
    return (static_cast<uint32_t>(idx) < m_uiNumObjects) ? m_apObjects[idx] : SceneObjectPtr::Null();
  }

  uint32_t GetNumObjects() const { return m_uiNumObjects; }

  /// allows for casting an arbitrary ray and finding out what it hits (if anything)
  const SceneObjectPtr& TestRayHit( const SceneRay& ray ) const;

  /// processes pending removals
  /// clears ray hit results and queued interactions from previous frame
  /// caches ray hits from finger/tool (pointable) pointing.
  /// queues new set of interactions for polling/processing.
  /// the queued interactions will need to be polled/processed
  /// following Update().  See utility function
  /// DefaultProcessSceneInteractions() below for an example.
  LEAP_EXPORT void Update( const Frame& frame, float fDeltaTimeSeconds );

  /// does not queue interactions, sets all objects to deselected state immediately.
  LEAP_EXPORT void DeselectAll();

  /// the frame transform and scale are used to modify positions and directions gathered from
  /// leap frame method calls (e.g. frame.fingers()[0].position()).
  /// this allows for mapping spatial data from the leap device to your scene space.
  /// for example if your scene world is entirely within the cube (-1, -1, -1) (1, 1, 1)
  /// you'll want to set the scale factor to something like 1/400.
  /// if the you want to offset or rotate the interaction of the hand with the scene you'll use the frame
  /// transform.
  void SetFrameTransform( const Matrix& mtxFrameTransform ) { m_mtxFrameTransform = mtxFrameTransform; }

  const Matrix& GetFrameTransform() const { return m_mtxFrameTransform; }

  void SetFrameScale( float fFrameScale ) { m_fFrameScale = fFrameScale; }

  float GetFrameScale() const { return m_fFrameScale; }

  /// a radius to use for spheres placed at finger/tool tip locations when
  /// checking for contact with objects in the scene.  analogous to size of a finger tip.
  void SetPointableRadius( float fRadius ) { m_fPointableRadius = fRadius; }

  float GetPointableRadius() const { return m_fPointableRadius; }

  /// the amount of time an object must be pointed at or touched before it is selected
  void SetSelectHitTime( float fSelectHitTime ) { m_fSelectHitTime = fSelectHitTime; }

  float GetSelectHitTime() const { return m_fSelectHitTime; }

  /// generic user data that can be associated with the scene.
  /// no management of any kind is done with it.
  void SetUserData( void* pUserData ) { m_pUserData = pUserData; }

  void* GetUserData() const { return m_pUserData; }

  /// the last value of the fDeltatTimeSeconds argument passed to Update()
  float GetDeltaTime() const { return m_fDeltaTimeSeconds; }

  /// the number of ray tests that resulted in hits from the last update
  uint32_t GetNumRayHits() const { return m_uiNumRayHits; }

  /// access to a ray test hit that happened during the last update
  const SceneRayHit* GetRayHit( uint32_t idx ) const
  {
    return idx < m_uiNumRayHits ? &(m_aRayHits[idx]) : NULL;
  }

  /// number of potential interactions that were queued up during the last update
  uint32_t GetNumQueuedInteractions() const { return m_uiNumQueuedInteractions; }

  /// access to a potential interaction queued up during the last update
  const SceneInteraction* GetQueuedInteraction( uint32_t idx ) const
  {
    return idx < m_uiNumQueuedInteractions ? &(m_aInteractionQueue[idx]) : NULL;
  }

  /// transforms a point from the Leap API (e.g. Pointable::tipPosition()) into scene space
  Vector TransformFramePoint( const Vector& vFramePoint )
  {
    return m_mtxFrameTransform.transformPoint( vFramePoint * m_fFrameScale );
  }

  /// transform a direction from the LeapAPI (e.g. Pointable::direction()) into scene space
  Vector TransformFrameDirection( const Vector& vFrameDirection )
  {
    return m_mtxFrameTransform.transformDirection( vFrameDirection );
  }

  uint32_t GetFlags() const { return m_uiFlags; }

  bool GetUpdateContact() const { return (m_uiFlags & kF_UpdateContact) != 0; }

  void SetUpdateContact( bool bUpdateContact )
  {
    m_uiFlags = bUpdateContact ? (m_uiFlags | kF_UpdateContact) : (m_uiFlags & ~kF_UpdateContact);
  }

  bool GetUpdateRayCast() const { return (m_uiFlags & kF_UpdateRayCast) != 0; }

  void SetUpdateRayCast( bool bUpdateRayCast )
  {
    m_uiFlags = bUpdateRayCast ? (m_uiFlags | kF_UpdateRayCast) : (m_uiFlags & ~kF_UpdateRayCast);
  }

  // internal methods for Scene
private:
  void updateSelectionAndContact( const Frame& frame );

  void queueDeselectAll();

  bool queueInteraction( const SceneInteraction& interaction )
  {
    if ( m_uiNumQueuedInteractions < static_cast<uint32_t>(kInteractionQueueLength) )
    {
      m_aInteractionQueue[m_uiNumQueuedInteractions++] = interaction;
    }

    return false;
  }

  void clearInteractionQueue()
  {
    // release object references from the interaction queue
    for ( uint32_t i = 0; i < m_uiNumQueuedInteractions; m_aInteractionQueue[i++].m_pObject.Release() );
    m_uiNumQueuedInteractions = 0;
  }

  void clearRayHits()
  {
    // release object references from the ray hits
    for ( uint32_t i = 0; i < m_uiNumRayHits; m_aRayHits[i++].m_pHitObject.Release() );
    m_uiNumRayHits = 0;
  }

  void updateInteraction( const Frame& frame );

  void processPendingRemovals();

  bool testRayHitClosest( SceneRayHit& hitResult );

  void updateContact( const SceneContactPoint& testPoint );

  template<class T>
  T* allocateObject()
  {
    if ( m_uiNumObjects < static_cast<uint32_t>(kMaxObjects) )
    {
      if ( T* pObject = new T() )
      {
        pObject->m_pScene           = this;
        pObject->m_serial           = m_uiNextSerial++;
        pObject->m_index            = static_cast<uint16_t>(m_uiNumObjects);
        m_apObjects[m_uiNumObjects++] = SceneObjectPtr(pObject);

        return pObject;
      }
    }

    return NULL;
  }

  void deallocateObject( uint32_t idxToRemove );

private:
  void*                   m_pUserData;
  float                   m_fDeltaTimeSeconds;
  float                   m_fPointableRadius;
  float                   m_fSelectHitTime;

  Matrix                  m_mtxFrameTransform;
  float                   m_fFrameScale;

  SceneObjectPtr          m_apObjects[kMaxObjects];
  SceneRayHit             m_aRayHits[kMaxRayHits];
  SceneInteraction        m_aInteractionQueue[kInteractionQueueLength];

  uint32_t                m_uiNumObjects;
  uint32_t                m_uiNumRayHits;
  uint32_t                m_uiNumQueuedInteractions;
  uint32_t                m_uiNumPendingRemovals;
  uint32_t                m_uiNextSerial;
  uint32_t                m_uiFlags;
}; // Scene

/// type identifier for scene objects
/// only the base class type is explicitly defined.
/// additional values come from SceneObject::NextObjectType()
/// note: object types are determined lazily.
/// once queried the value will never change, but the run-time order in which they are queried
/// determines the value.  there is no guarantee that
/// the type id of an inheriting class will be larger than the type id of the superclass.
/// the only guarantee is that it will be different.
/// see implementations of SceneBox::ObjectType() and SceneBox::GetType()
enum eSceneObjectType
{
  kSOT_SceneObject = 0,

  // force 32 bit size
  kSOT_Invalid = 0x7fffffff
};

/// abstract base class of objects managed by a Scene.
/// The only interface used by Scene, it has no knowledge
/// of concrete implementations.
class LEAP_EXPORT_CLASS SceneObject
{
  friend class Scene;

public:
  static eSceneObjectType ObjectType() { return kSOT_SceneObject; }

  static eSceneObjectType NextObjectType()
  {
    static uint32_t s_nextID = kSOT_SceneObject + 1;
    return static_cast<eSceneObjectType>( s_nextID + 1 );
  }

  enum { kMaxContactPoints = 5 };

public:
  SceneObject()
    : m_pUserData(NULL),
      m_paContactPoints( m_aContactPoints ),
      m_paLastContactPoints( m_aContactPoints + kMaxContactPoints ),
      m_fTotalHitTime(0.0f),
      m_fScale(1.0f),
      m_uiNumPointing(0),
      m_uiNumContacts(0),
      m_uiLastNumContacts(0),
      m_uiHasInitialContact(0),
      m_bSelected(false),
      m_bPendingRemoval(false),
      m_pScene(NULL)
  {}

  virtual ~SceneObject() {}

public:
  /// pure virtual methods requiring implementation in inheriting classes.
  LEAP_EXPORT virtual eSceneObjectType GetType() const = 0;

  LEAP_EXPORT virtual bool TestRayHit(const SceneRay& testRay, float& fHitDistOut) const = 0;

  LEAP_EXPORT virtual bool TestSphereHit(const Vector& vTestPoint, float fTestRadius) const = 0;

#if defined(LEAP_SCENE_USE_UTIL_GL)
  LEAP_EXPORT virtual void DebugDrawGL( LeapUtilGL::eStyle drawStyle=LeapUtilGL::kStyle_Solid ) const = 0;
#endif

  template<class T>
  T* GetAs()
  {
#if defined(LEAP_SCENE_NO_DYNAMIC_CAST)
    // When relying on simple internal type ids GetAs() only works for exact type matches or up casts to the base type.
    // this mechanism is provided as a simple alternative for code bases that prohibit the use of dynamic_cast.
    // a complete, hierarchically correct type id system is beyond the scope of this code.
    return (this && (GetType() == T::ObjectType() || GetType() == SceneObject::ObjectType())) ? static_cast<T*>(this) : NULL;
#else
    return dynamic_cast<T*>(this);
#endif
  }

  template<class T>
  const T* GetAs() const
  {
    return const_cast<SceneObject*>(this)->GetAs<T>();
  }

  const SceneObjectPtr& GetSceneObjectPtr() const
  {
    return (this && m_pScene) ? m_pScene->GetObjectByIndex(m_index) : SceneObjectPtr::Null();
  }

  operator const SceneObjectPtr&() const
  {
    return GetSceneObjectPtr();
  }

  /// a unique serial number assigned to each object at creation time.
  uint32_t GetSerial() const { return m_serial; }

  Scene* GetScene() const { return m_pScene; }

  void Translate(const Vector& translation) { m_mtxTransform.origin += translation; }

  void Rotate(const Vector& axis, float angleRadians)
  {
    m_mtxTransform =  m_mtxTransform * Matrix(axis, angleRadians);
  }

  void Rotate(const Matrix& rotationMatrix)
  {
    m_mtxTransform = m_mtxTransform * LeapUtil::ExtractRotation( rotationMatrix );
  }

  void Scale(float scaleMult)
  {
    m_fScale *= scaleMult;
  }

  void Transform( const Matrix& mtxTransform )
  {
    m_mtxTransform = m_mtxTransform * mtxTransform;
  }

  bool ApplyInteraction( const SceneInteraction& interaction )
  {
    if ( IsPendingRemoval() )
    {
      return false;
    }

    if ( interaction.HasRotation() )
    {
      Rotate( interaction.GetRotation() );
    }

    if ( interaction.HasTranslation() )
    {
      Translate( interaction.GetTranslation() );
    }

    if ( interaction.HasScale() )
    {
      Scale( interaction.GetScale() );
    }

    if ( interaction.HasSelectionChange() )
    {
      SetSelected( interaction.IsSelected() );
    }

    return true;
  }

  void SetCenter(const Vector& vCenter) { m_mtxTransform.origin = vCenter; }

  void SetRotation(const Vector& vAxis, float fAngleRadians)
  {
    m_mtxTransform.setRotation( vAxis, fAngleRadians );
  }

  void SetRotation(const Matrix& rotationMatrix)
  {
    m_mtxTransform = Matrix( rotationMatrix.xBasis, rotationMatrix.yBasis, rotationMatrix.zBasis, m_mtxTransform.origin );
  }

  void SetScale(float scale) { m_fScale = scale; }

  const Vector& GetCenter() const { return m_mtxTransform.origin; }

  const Matrix GetRotation() const { return Matrix( m_mtxTransform.xBasis, m_mtxTransform.yBasis, m_mtxTransform.zBasis ); }

  const Matrix& GetTransform() const { return m_mtxTransform; }

  float GetScale() const { return m_fScale; }

  bool IsSelected() const { return m_bSelected != false; }

  void SetSelected(bool selected)
  {
    m_bSelected = selected;
    if ( !m_bSelected )
    {
      ClearHitTime();
    }
  }

  uint32_t GetNumContacts() const { return m_uiNumContacts; }

  uint32_t GetNumPointing() const { return m_uiNumPointing; }

  uint32_t GetLastNumContacts() const { return m_uiLastNumContacts; }

  bool HasInitialContact() const { return m_uiHasInitialContact != 0; }

  void ClearNumContacts() { m_uiNumContacts = 0; }

  void ClearNumPointing() { m_uiNumPointing = 0; }

  void ClearHitTime() { m_fTotalHitTime = 0.0f; }

  void ClearInitialContact() { m_uiHasInitialContact = 0; }

  void IncNumPointing() { m_uiNumPointing++; }

  void IncNumContacts(const SceneContactPoint& contactPoint)
  {
    if (m_uiNumContacts < kMaxContactPoints)
    {
      m_paContactPoints[m_uiNumContacts++] = contactPoint;
    }
  }

  void ClearHits()
  {
    m_uiNumContacts       = 0;
    m_uiLastNumContacts   = 0;
    m_uiHasInitialContact = 0;
    m_uiNumPointing       = 0;
    m_fTotalHitTime       = 0.0f;
  }

  float GetTotalHitTime() const { return m_fTotalHitTime; }

  const SceneContactPoint* GetContactPoint(uint32_t uiIndex) const
  {
    return uiIndex < static_cast<uint32_t>(kMaxContactPoints) ? m_paContactPoints + uiIndex : NULL;
  }

  const SceneContactPoint* GetLastContactPoint(uint32_t uiIndex) const
  {
    return uiIndex < static_cast<uint32_t>(kMaxContactPoints) ? m_paLastContactPoints + uiIndex : NULL;
  }

  const SceneContactPoint* GetInitialContactPoint() const
  {
    return m_uiHasInitialContact ? &m_initialContactPoint : NULL;
  }

  const SceneContactPoint* GetContactPointByPointableID(int iPointableID) const
  {
    for ( uint32_t i = 0; i < m_uiNumContacts; i++ )
    {
      if ( m_paContactPoints[i].m_iPointableID == iPointableID )
      {
        return m_paContactPoints + i;
      }
    }

    return NULL;
  }

  const SceneContactPoint* GetLastContactPointByPointableID(int iPointableID) const
  {
    for ( uint32_t i = 0; i < m_uiLastNumContacts; i++ )
    {
      if ( m_paLastContactPoints[i].m_iPointableID == iPointableID )
      {
        return m_paContactPoints + i;
      }
    }

    return NULL;
  }

  void* GetUserData() const { return m_pUserData; }

  void SetUserData( void* pUserData ) { m_pUserData = pUserData; }

  bool IsPendingRemoval() const { return m_bPendingRemoval != false; }

  /// returns a transform for converting world space coordinates and direction to object space.
  Matrix GetWorldToObjectTransform() const { return LeapUtil::RigidInverse(m_mtxTransform); }

  Vector WorldToObjectPoint( const Vector& vPoint ) const
  {
    return LeapUtil::RigidInverse(m_mtxTransform).transformPoint( vPoint );
  }

protected:
  void rotateContactPoints()
  {
    m_uiLastNumContacts  = m_uiNumContacts;
    m_uiNumContacts      = 0;

    SceneContactPoint* pTemp  = m_paLastContactPoints;
    m_paLastContactPoints     = m_paContactPoints;
    m_paContactPoints         = pTemp;
  }

protected:
  Matrix              m_mtxTransform;
  SceneContactPoint   m_initialContactPoint;
  SceneContactPoint   m_aContactPoints[kMaxContactPoints*2];
  void*               m_pUserData;
  SceneContactPoint*  m_paContactPoints;
  SceneContactPoint*  m_paLastContactPoints;
  float               m_fTotalHitTime;
  float               m_fScale;

  uint8_t             m_uiNumPointing;
  uint8_t             m_uiNumContacts;
  uint8_t             m_uiLastNumContacts;
  uint8_t             m_uiHasInitialContact;

  uint8_t             m_bSelected;

private:
  uint8_t             m_bPendingRemoval;
  uint16_t            m_index;
  uint32_t            m_serial;
  Scene*              m_pScene;
}; // SceneObject


/// after calling Scene::Update()
/// either call this function or implement your own version with
/// additional logic for your objects.
inline void DefaultProcessSceneInteractions( Scene& scene )
{
  for ( uint32_t i = 0, n = scene.GetNumQueuedInteractions(); i < n; i++ )
  {
    const SceneInteraction& interaction = *scene.GetQueuedInteraction( i );
    interaction.GetObject()->ApplyInteraction( interaction );
  }
}

class LEAP_EXPORT_CLASS SceneBox : public SceneObject
{
public:
  /// if you extend SceneObject or any of its descendant classes
  /// the public methods ObjectType() and GetType() should be implemented exactly as they are below.
  /// they have not been encapsulated in an implementation macro to maintain clarity and ease of debugging.
  static eSceneObjectType ObjectType() { static const eSceneObjectType s_type = NextObjectType(); return s_type; }

  LEAP_EXPORT virtual eSceneObjectType GetType() const { return ObjectType(); }

  SceneBox()
    : m_vSize( 1, 1, 1 )
  {}

  LEAP_EXPORT virtual ~SceneBox() {}

  const Vector& GetSize() const { return m_vSize; }

  void SetSize( const Vector& vSize ) { m_vSize = vSize; }

  LEAP_EXPORT virtual bool TestRayHit(const SceneRay& testRay, float& fHitDistOut) const;

  LEAP_EXPORT virtual bool TestSphereHit(const Vector& vTestPoint, float fTestRadius) const;

#if defined(LEAP_SCENE_USE_UTIL_GL)
  LEAP_EXPORT virtual void DebugDrawGL( LeapUtilGL::eStyle drawStyle=LeapUtilGL::kStyle_Solid ) const;
#endif

private:
  Vector m_vSize;
}; // SceneBox

class LEAP_EXPORT_CLASS SceneCylinder : public SceneObject
{
public:
  /// if you extend SceneObject or any of its descendant classes
  /// the public methods ObjectType() and GetType() should be implemented exactly as they are below.
  /// they have not been encapsulated in an implementation macro to maintain clarity and ease of debugging.
  static eSceneObjectType ObjectType() { static const eSceneObjectType s_type = NextObjectType(); return s_type; }

  LEAP_EXPORT virtual eSceneObjectType GetType() const { return ObjectType(); }

  SceneCylinder() : m_fRadius( 1.0f ), m_fHeight( 1.0f ) {}

  virtual ~SceneCylinder() {}

  void SetRadius(float radius) { m_fRadius = radius; }

  void SetHeight(float height) { m_fHeight = height; }

  const Vector& GetAxis() const { return m_mtxTransform.yBasis; }

  float GetRadius() const { return m_fRadius; }

  float GetHeight() const { return m_fHeight; }

  LEAP_EXPORT virtual bool TestRayHit(const SceneRay& testRay, float& fHitDistOut) const;

  LEAP_EXPORT virtual bool TestSphereHit(const Vector& vTestPoint, float fTestRadius) const;

#if defined(LEAP_SCENE_USE_UTIL_GL)
  LEAP_EXPORT virtual void DebugDrawGL( LeapUtilGL::eStyle drawStyle=LeapUtilGL::kStyle_Solid ) const;
#endif

private:
  float    m_fRadius;
  float    m_fHeight;
}; // SceneCylinder

class LEAP_EXPORT_CLASS SceneDisk : public SceneObject
{
public:
  /// if you extend SceneObject or any of its descendant classes
  /// the public methods ObjectType() and GetType() should be implemented exactly as they are below.
  /// they have not been encapsulated in an implementation macro to maintain clarity and ease of debugging.
  static eSceneObjectType ObjectType() { static const eSceneObjectType s_type = NextObjectType(); return s_type; }

  LEAP_EXPORT virtual eSceneObjectType GetType() const { return ObjectType(); }

  SceneDisk() : m_fRadius( 1.0f ) {}

  virtual ~SceneDisk() {}

  void SetRadius(float radius) { m_fRadius = radius; }

  const Vector& GetNormal() const { return m_mtxTransform.zBasis; }

  float GetRadius() const { return m_fRadius; }

  LEAP_EXPORT virtual bool TestRayHit(const SceneRay& testRay, float& fHitDistOut) const;

  LEAP_EXPORT virtual bool TestSphereHit(const Vector& vTestPoint, float fTestRadius) const;

#if defined(LEAP_SCENE_USE_UTIL_GL)
  LEAP_EXPORT virtual void DebugDrawGL( LeapUtilGL::eStyle drawStyle=LeapUtilGL::kStyle_Solid ) const;
#endif

private:
  float    m_fRadius;
}; // SceneDisk

class LEAP_EXPORT_CLASS ScenePlane : public SceneObject
{
public:
  /// if you extend SceneObject or any of its descendant classes
  /// the public methods ObjectType() and GetType() should be implemented exactly as they are below.
  /// they have not been encapsulated in an implementation macro to maintain clarity and ease of debugging.
  static eSceneObjectType ObjectType() { static const eSceneObjectType s_type = NextObjectType(); return s_type; }

  LEAP_EXPORT virtual eSceneObjectType GetType() const { return ObjectType(); }

  ScenePlane() {}

  virtual ~ScenePlane() {}

  const Vector& GetNormal() const { return m_mtxTransform.zBasis; }

  LEAP_EXPORT virtual bool TestRayHit(const SceneRay& testRay, float& fHitDistOut) const;

  LEAP_EXPORT virtual bool TestSphereHit(const Vector& vPoint, float fRadius) const;

#if defined(LEAP_SCENE_USE_UTIL_GL)
  LEAP_EXPORT virtual void DebugDrawGL( LeapUtilGL::eStyle drawStyle=LeapUtilGL::kStyle_Solid ) const;
#endif
}; // ScenePlane

class LEAP_EXPORT_CLASS SceneSphere : public SceneObject
{
public:
  /// if you extend SceneObject or any of its descendant classes
  /// the public methods ObjectType() and GetType() should be implemented exactly as they are below.
  /// they have not been encapsulated in an implementation macro to maintain clarity and ease of debugging.
  static eSceneObjectType ObjectType() { static const eSceneObjectType s_type = NextObjectType(); return s_type; }

  LEAP_EXPORT virtual eSceneObjectType GetType() const { return ObjectType(); }

  SceneSphere() : m_fRadius(1.0f) { }

  virtual ~SceneSphere() {}

  void SetRadius(const float& radius) { m_fRadius = radius; }

  float GetRadius() const { return m_fRadius; }

  LEAP_EXPORT virtual bool TestRayHit(const SceneRay& testRay, float& fHitDistOut) const;

  LEAP_EXPORT virtual bool TestSphereHit(const Vector& vTestCenter, float fTestRadius) const;

#if defined(LEAP_SCENE_USE_UTIL_GL)
  LEAP_EXPORT virtual void DebugDrawGL( LeapUtilGL::eStyle drawStyle=LeapUtilGL::kStyle_Solid ) const;
#endif

private:
  float      m_fRadius;
}; // SceneSphere

}; // namespace Leap

#endif // __LeapScene_h__
