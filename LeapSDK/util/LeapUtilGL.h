/******************************************************************************\
* Copyright (C) Leap Motion, Inc. 2011-2013.                                   *
* Leap Motion proprietary and  confidential.  Not for distribution.            *
* Use subject to the terms of the Leap Motion SDK Agreement available at       *
* https://developer.leapmotion.com/sdk_agreement, or another agreement between *
* Leap Motion and you, your company or other organization.                     *
\******************************************************************************/

#ifndef __LeapUtilGL_h__
#define __LeapUtilGL_h__

#if !defined(__GL_H__)
  #if defined(WIN32)
    #include <windows.h>
    #include <GL/gl.h>
  #elif defined(__MACH__)
    #include <OpenGL/gl.h>
  #else
    #include <GL/gl.h>
  #endif
#endif // !__GL_H__

#include "LeapUtil.h"

namespace LeapUtilGL {

/// all drawing utilities draw unit sized objects centered at the origin with no rotation
/// in the current GL drawing context (orientation, scaling, lighting configuration, etc).
/// use glTranslatef() to change position, glScalef() to change size.
/// these object draw without any side effects to the GL state.

enum eAxis { kAxis_X, kAxis_Y, kAxis_Z };

enum ePlane { kPlane_XY, kPlane_YZ, kPlane_ZX };

enum eStyle { kStyle_Outline, kStyle_Solid };

/// grid is drawn with unlit colored lines.
void drawGrid( ePlane plane, unsigned int horizSubDivs, unsigned int vertSubDivs );

void drawSphere( eStyle style );

/// quad is double-sided.
void drawQuad( eStyle style, ePlane plane );

void drawBox( eStyle style );

void drawCylinder( eStyle style, eAxis axis );

/// disk is double-side
void drawDisk( eStyle style, ePlane plane );

/// arrow is drawn with unlit colored lines.
void drawArrow( eAxis axis );

void drawAxes();

/// convenience structure for easy conversion from Leap::Vector to const float*
/// vector passed to OpenGL fv functions.  convert using Leap::Vector::ToVector4<GLVector4fv>()
/// can also be used when 3 or 4 floating point values are expected.
struct GLVector4fv
{
  GLVector4fv() : x(0), y(0), z(0), w(0) {}
  GLVector4fv( float _x, float _y, float _z, float _w=1.0f ) : x(_x), y(_y), z(_z), w(_w) {}
  GLVector4fv( const Leap::Vector& vRHS, float _w=1.0f ) : x(vRHS.x), y(vRHS.y), z(vRHS.z), w(_w) {}
  static const GLVector4fv& One() { static const GLVector4fv s_one(1.0f, 1.0f, 1.0f, 1.0f); return s_one; }
  operator const GLfloat*() const { return &x; }

  GLfloat x, y, z, w;
};

/// additional sphere/cylinder drawing utilities
void drawSphere(eStyle style, const Leap::Vector& vCenter, float fRadius);

void drawCylinder(eStyle style, const Leap::Vector& vBottom, const Leap::Vector& vTop, float fRadius);

/// utility for drawing a skeleton API hand as seen in diagnostic visualizer
void drawSkeletonHand(const Leap::Hand& hand, const GLVector4fv& vBoneColor = GLVector4fv::One(), const GLVector4fv& vJointColor = GLVector4fv::One());

/// utility class for keeping the gl matrix stack push/pop operations paired up correctly.
class GLMatrixScope
{
public:
  GLMatrixScope() { glPushMatrix(); }
  ~GLMatrixScope() { glPopMatrix(); }

private:
  // some compilers don't like zero sized objects.
  int m_dummy;
};

/// utility class for caching and restoring GL attributes via the glPushAttrib/glPopAttrib calls.
class GLAttribScope
{
public:
  GLAttribScope( GLbitfield flags )
  {
    glPushAttrib( flags );
  }

  ~GLAttribScope()
  {
    glPopAttrib();
  }

private:
  // some compilers don't like zero sized objects.
  int m_dummy;
};

class CameraGL : public LeapUtil::Camera
{
public:
  CameraGL() {}

  static void ResetGLProjection()
  {
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
  }

  static void ResetGLView()
  {
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();
  }

  void SetupGLProjection() const;

  void SetupGLView() const;
};

}

#endif // __LeapUtilGL_h__
