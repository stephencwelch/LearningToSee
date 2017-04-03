/******************************************************************************\
* Copyright (C) Leap Motion, Inc. 2011-2013.                                   *
* Leap Motion proprietary and  confidential.  Not for distribution.            *
* Use subject to the terms of the Leap Motion SDK Agreement available at       *
* https://developer.leapmotion.com/sdk_agreement, or another agreement between *
* Leap Motion and you, your company or other organization.                     *
\******************************************************************************/

#include "LeapUtilGL.h"

#if !defined(__GLU_H__)
  #if defined(WIN32)
    #include <windows.h>
    #include <GL/glu.h>
  #elif defined(__MACH__)
    #include <OpenGL/glu.h>
  #else
    #include <GL/glu.h>
  #endif
#endif // !__GL_H__

namespace LeapUtilGL {

using namespace Leap;
using namespace LeapUtil;

// create/destroy single global quadric instance for drawing curved surfaces with GLU
class Quadric
{
public:
  Quadric() {}

  ~Quadric()
  {
    if ( s_pQuadric )
    {
      gluDeleteQuadric( s_pQuadric );
      s_pQuadric = NULL;
    }
  }

  operator GLUquadric* ()
  {
    if ( !s_pQuadric )
    {
      s_pQuadric = gluNewQuadric();
    }

    return s_pQuadric;
  }

  static GLUquadric* s_pQuadric;
};

GLUquadric* Quadric::s_pQuadric = NULL;

static Quadric s_quadric;

void drawGrid( ePlane plane, unsigned int horizSubdivs, unsigned int vertSubdivs )
{
    const float fHalfGridSize   = 0.5f;
    const float fHGridStep      = (fHalfGridSize + fHalfGridSize)/static_cast<float>(Max(horizSubdivs, 1u));
    const float fVGridStep      = (fHalfGridSize + fHalfGridSize)/static_cast<float>(Max(vertSubdivs, 1u));
    const float fHEndStep       = fHalfGridSize + fHGridStep;
    const float fVEndStep       = fHalfGridSize + fVGridStep;

    GLAttribScope lightingScope( GL_LIGHTING_BIT );

    glDisable(GL_LIGHTING);

    glBegin( GL_LINES );

    switch ( plane )
    {
      case kPlane_XY:
        for ( float x = -fHalfGridSize; x < fHEndStep; x += fHGridStep )
        {
          glVertex3f( x, -fHalfGridSize, 0 );
          glVertex3f( x, fHalfGridSize, 0 );
        }

        for ( float y = -fHalfGridSize; y < fVEndStep; y += fVGridStep )
        {
          glVertex3f( -fHalfGridSize, y, 0 );
          glVertex3f( fHalfGridSize, y, 0 );
        }
        break;

      case kPlane_YZ:
        for ( float y = -fHalfGridSize; y < fHEndStep; y += fHGridStep )
        {
          glVertex3f( 0, y, -fHalfGridSize );
          glVertex3f( 0, y, fHalfGridSize );
        }

        for ( float z = -fHalfGridSize; z < fVEndStep; z += fVGridStep )
        {
          glVertex3f( 0, -fHalfGridSize, z );
          glVertex3f( 0, fHalfGridSize, z );
        }
        break;

      case kPlane_ZX:
        for ( float z = -fHalfGridSize; z < fHEndStep; z += fHGridStep )
        {
          glVertex3f( -fHalfGridSize, 0, z );
          glVertex3f( fHalfGridSize,  0, z );
        }

        for ( float x = -fHalfGridSize; x < fVEndStep; x += fVGridStep )
        {
          glVertex3f( x, 0, -fHalfGridSize );
          glVertex3f( x,  0, fHalfGridSize );
        }
        break;
    }

    glEnd();
}

void drawSphere( eStyle style )
{
  switch ( style )
  {
   case kStyle_Outline:
    gluQuadricDrawStyle(s_quadric, GLU_SILHOUETTE);
    glPushAttrib( GL_LIGHTING_BIT );
    glDisable(GL_LIGHTING);
    break;

   case kStyle_Solid:
    gluQuadricDrawStyle(s_quadric, GLU_FILL);
    break;
  }

  gluSphere( s_quadric, 1.0, 32, 32 );

  switch ( style )
  {
   case kStyle_Outline:
    glPopAttrib();
    break;

   case kStyle_Solid:
    break;
  }
}

void drawQuad( eStyle style, ePlane plane )
{
  switch ( style )
  {
   case kStyle_Outline:
    glPushAttrib( GL_LIGHTING_BIT );
    glDisable(GL_LIGHTING);
    break;

   case kStyle_Solid:
    break;
  }

  switch ( plane )
  {
  case kPlane_XY:
    break;
  case kPlane_YZ:
    glPushMatrix();
    glRotatef( 90.0f, 0, 1, 0 );
    break;
  case kPlane_ZX:
    glPushMatrix();
    glRotatef( -90.0f, 1, 0, 0 );
    break;
  }

  const float kfHalfSize = 0.5f;

  glBegin( style == kStyle_Outline ? GL_LINE_LOOP : GL_TRIANGLES );
  glNormal3f( 0, 0, 1 );
  glTexCoord2f( 0, 0 );
  glVertex3f( -kfHalfSize,  -kfHalfSize, 0 );
  glTexCoord2f( 1, 0 );
  glVertex3f(  kfHalfSize,  -kfHalfSize, 0 );
  glTexCoord2f( 1, 1 );
  glVertex3f(  kfHalfSize,   kfHalfSize, 0 );
  glEnd();

  glBegin( style == kStyle_Outline ? GL_LINE_LOOP : GL_TRIANGLES );
  glTexCoord2f( 1, 1 );
  glVertex3f(  kfHalfSize,   kfHalfSize, 0 );
  glTexCoord2f( 0, 1 );
  glVertex3f( -kfHalfSize,   kfHalfSize, 0 );
  glTexCoord2f( 0, 0 );
  glVertex3f( -kfHalfSize,  -kfHalfSize, 0 );
  glEnd();

  glBegin( style == kStyle_Outline ? GL_LINE_LOOP : GL_TRIANGLES );
  glNormal3f( 0, 0, -1 );
  glTexCoord2f( 0, 0 );
  glVertex3f(  kfHalfSize,  -kfHalfSize, 0 );
  glTexCoord2f( 1, 0 );
  glVertex3f( -kfHalfSize,  -kfHalfSize, 0 );
  glTexCoord2f( 1, 1 );
  glVertex3f( -kfHalfSize,   kfHalfSize, 0 );
  glEnd();

  glBegin( style == kStyle_Outline ? GL_LINE_LOOP : GL_TRIANGLES );
  glTexCoord2f( 1, 1 );
  glVertex3f( -kfHalfSize,   kfHalfSize, 0 );
  glTexCoord2f( 0, 1 );
  glVertex3f(  kfHalfSize,   kfHalfSize, 0 );
  glTexCoord2f( 0, 0 );
  glVertex3f(  kfHalfSize,  -kfHalfSize, 0 );
  glEnd();

  switch ( plane )
  {
  case kPlane_XY:
    break;
  case kPlane_YZ:
  case kPlane_ZX:
    glPopMatrix();
    break;
  }

  switch ( style )
  {
  case kStyle_Outline:
    glPopAttrib();
    break;
  case kStyle_Solid:
    break;
  }
}


void drawBox( eStyle style )
{
  static const float s_afCorners[8][3] = {
                                            // near face - ccw facing origin from face.
                                            {-0.5f, -0.5f,  0.5f},
                                            { 0.5f, -0.5f,  0.5f},
                                            { 0.5f,  0.5f,  0.5f},
                                            {-0.5f,  0.5f,  0.5f},

                                            // far face - ccw facing origin from face
                                            { 0.5f, -0.5f,  -0.5f},
                                            {-0.5f, -0.5f,  -0.5f},
                                            {-0.5f,  0.5f,  -0.5f},
                                            { 0.5f,  0.5f,  -0.5f},

 };

  switch ( style )
  {
   case kStyle_Outline:
    glPushAttrib( GL_LIGHTING_BIT );
    glDisable(GL_LIGHTING);
    break;

   case kStyle_Solid:
    break;
  }

  glBegin( style == kStyle_Solid ? GL_TRIANGLES : GL_LINE_LOOP );

  // near
  glNormal3f( 0, 0, 1 );
  glVertex3fv( s_afCorners[0] );
  glVertex3fv( s_afCorners[1] );
  glVertex3fv( s_afCorners[2] );

  glEnd();
  glBegin( style == kStyle_Solid ? GL_TRIANGLES : GL_LINE_LOOP );

  glVertex3fv( s_afCorners[2] );
  glVertex3fv( s_afCorners[3] );
  glVertex3fv( s_afCorners[0] );

  glEnd();
  glBegin( style == kStyle_Solid ? GL_TRIANGLES : GL_LINE_LOOP );

  // far
  glNormal3f( 0, 0, -1 );
  glVertex3fv( s_afCorners[4] );
  glVertex3fv( s_afCorners[5] );
  glVertex3fv( s_afCorners[6] );

  glEnd();
  glBegin( style == kStyle_Solid ? GL_TRIANGLES : GL_LINE_LOOP );

  glVertex3fv( s_afCorners[6] );
  glVertex3fv( s_afCorners[7] );
  glVertex3fv( s_afCorners[4] );

  glEnd();

  glBegin( style == kStyle_Solid ? GL_TRIANGLES : GL_LINE_LOOP );

  // right
  glNormal3f( 1, 0, 0 );
  glVertex3fv( s_afCorners[1] );
  glVertex3fv( s_afCorners[4] );
  glVertex3fv( s_afCorners[7] );

  glEnd();
  glBegin( style == kStyle_Solid ? GL_TRIANGLES : GL_LINE_LOOP );

  glVertex3fv( s_afCorners[7] );
  glVertex3fv( s_afCorners[2] );
  glVertex3fv( s_afCorners[1] );

  glEnd();
  glBegin( style == kStyle_Solid ? GL_TRIANGLES : GL_LINE_LOOP );

  // left
  glNormal3f( -1, 0, 0 );
  glVertex3fv( s_afCorners[5] );
  glVertex3fv( s_afCorners[0] );
  glVertex3fv( s_afCorners[3] );

  glEnd();
  glBegin( style == kStyle_Solid ? GL_TRIANGLES : GL_LINE_LOOP );

  glVertex3fv( s_afCorners[3] );
  glVertex3fv( s_afCorners[6] );
  glVertex3fv( s_afCorners[5] );

  glEnd();

  glBegin( style == kStyle_Solid ? GL_TRIANGLES : GL_LINE_LOOP );

  // bottom
  glNormal3f( 0, -1, 0 );
  glVertex3fv( s_afCorners[0] );
  glVertex3fv( s_afCorners[5] );
  glVertex3fv( s_afCorners[4] );

  glEnd();
  glBegin( style == kStyle_Solid ? GL_TRIANGLES : GL_LINE_LOOP );

  glVertex3fv( s_afCorners[4] );
  glVertex3fv( s_afCorners[1] );
  glVertex3fv( s_afCorners[0] );

  glEnd();
  glBegin( style == kStyle_Solid ? GL_TRIANGLES : GL_LINE_LOOP );

  // top
  glNormal3f( 0, 1, 0 );
  glVertex3fv( s_afCorners[3] );
  glVertex3fv( s_afCorners[2] );
  glVertex3fv( s_afCorners[7] );

  glEnd();
  glBegin( style == kStyle_Solid ? GL_TRIANGLES : GL_LINE_LOOP );

  glVertex3fv( s_afCorners[7] );
  glVertex3fv( s_afCorners[6] );
  glVertex3fv( s_afCorners[3] );

  glEnd();

  switch ( style )
  {
   case kStyle_Outline:
    glPopAttrib();
    break;

   case kStyle_Solid:
    break;
  }
}

void drawCylinder( eStyle style, eAxis axis )
{
  GLMatrixScope matrixScope;

  switch ( style )
  {
   case kStyle_Outline:
    gluQuadricDrawStyle(s_quadric, GLU_SILHOUETTE);
    glPushAttrib( GL_LIGHTING_BIT );
    glDisable(GL_LIGHTING);
    break;

   case kStyle_Solid:
    gluQuadricDrawStyle(s_quadric, GLU_FILL);
    break;
  }

  switch ( axis )
  {
  case kAxis_X:
    glRotatef( 90.0f, 0, 1, 0 );
    break;
  case kAxis_Y:
    glRotatef( 90.0f, 1, 0, 0 );
    break;
  case kAxis_Z:
    break;
  }

  // draw end caps
  if ( style != kStyle_Outline )
  {
    GLMatrixScope matrixScope;

    glTranslatef( 0, 0, 0.5f );
    drawDisk( style, kPlane_XY );

    glTranslatef( 0, 0, -1.0f );
    drawDisk( style, kPlane_XY );
  }

  glTranslatef( 0, 0, -0.5f );

  gluCylinder( s_quadric, 0.5f, 0.5f, 1.0f, 32, 32 );

  switch ( style )
  {
   case kStyle_Outline:
    glPopAttrib();
    break;

   case kStyle_Solid:
    break;
  }
}

void drawDisk( eStyle style, ePlane plane )
{
  GLMatrixScope matrixScope;

  switch ( style )
  {
   case kStyle_Outline:
    gluQuadricDrawStyle(s_quadric, GLU_SILHOUETTE);
    glPushAttrib( GL_LIGHTING_BIT );
    glDisable(GL_LIGHTING);
    break;

   case kStyle_Solid:
    gluQuadricDrawStyle(s_quadric, GLU_FILL);
    break;
  }

  switch ( plane )
  {
  case kPlane_XY:
    break;
  case kPlane_YZ:
    glRotatef( 90.0f, 0, 1, 0 );
    break;
  case kPlane_ZX:
    glRotatef( -90.0f, 1, 0, 0 );
    break;
  }

  gluDisk(s_quadric, 0, 0.5f, 32, 1);
  glRotatef( 180.0f, 0, 1, 0 );
  gluDisk(s_quadric, 0, 0.5f, 32, 1);

  switch ( style )
  {
   case kStyle_Outline:
    glPopAttrib();
    break;

   case kStyle_Solid:
    break;
  }
}

void drawArrow( eAxis axis )
{
  glBegin( GL_LINES );

  switch ( axis )
  {
  case kAxis_X:
    glVertex3f( 0, 0, 0 );
    glVertex3f( 1, 0, 0 );

    glVertex3f( 1, 0, 0 );
    glVertex3f( 0.8f, 0.2f, 0 );

    glVertex3f( 1, 0, 0 );
    glVertex3f( 0.8f, -0.2f, 0 );
    break;

  case kAxis_Y:
    glVertex3f( 0, 0, 0 );
    glVertex3f( 0, 1, 0 );

    glVertex3f( 0, 1, 0 );
    glVertex3f( 0, 0.8f, 0.2f );

    glVertex3f( 0, 1, 0 );
    glVertex3f( 0, 0.8f, -0.2f );
    break;

  case kAxis_Z:
    glVertex3f( 0, 0, 0 );
    glVertex3f( 0, 0, 1 );

    glVertex3f( 0, 0, 1 );
    glVertex3f( 0.2f, 0, 0.8f );

    glVertex3f( 0, 0, 1 );
    glVertex3f( -0.2f, 0, 0.8f );
    break;
  }

  glEnd();
}

  // draw unit length X, Y, Z axes in red, green and blue.
  void drawAxes()
  {
    GLAttribScope attribScope(GL_CURRENT_BIT | GL_LIGHTING_BIT | GL_DEPTH_BUFFER_BIT);

    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);

    glColor3f(1, 0, 0);
    drawArrow(kAxis_X);

    glColor3f(0, 1, 0);
    drawArrow(kAxis_Y);

    glColor3f(0, 0, 1);
    drawArrow(kAxis_Z);
  }

  // additional sphere/cylinder drawing utilities
  void drawSphere(eStyle style, const Vector& vCenter, float fRadius)
  {
    GLMatrixScope matrixScope;

    glTranslatef(vCenter.x, vCenter.y, vCenter.z);
    glScalef(fRadius, fRadius, fRadius);

    drawSphere(style);
  }

  void drawCylinder(eStyle style, const Vector& vBottom, const Vector& vTop, float fRadius)
  {
    GLMatrixScope matrixScope;

    Vector vCenter = (vBottom + vTop) * 0.5f;

    Vector vDir = (vTop - vBottom).normalized();
    Vector vRotAxis = Vector::zAxis().cross(vDir);
    float fRotAngle = Vector::zAxis().angleTo(vDir);
    float fHeight = vTop.distanceTo(vBottom);

    glTranslatef(vCenter.x, vCenter.y, vCenter.z);
    glRotatef(fRotAngle * RAD_TO_DEG, vRotAxis.x, vRotAxis.y, vRotAxis.z);
    glScalef(fRadius, fRadius, fHeight);

    drawCylinder(style, kAxis_Z);
  }

  // convenience function for drawing skeleton API hand
  void drawSkeletonHand(const Leap::Hand& hand, const GLVector4fv& vBoneColor, const GLVector4fv& vJointColor) {
    static const float kfJointRadiusScale = 0.75f;
    static const float kfBoneRadiusScale = 0.5f;
    static const float kfPalmRadiusScale = 1.15f;

    LeapUtilGL::GLAttribScope colorScope( GL_CURRENT_BIT );

    const Vector vPalm = hand.palmPosition();
    const Vector vPalmDir = hand.direction();
    const Vector vPalmNormal = hand.palmNormal();
    const Vector vPalmSide = vPalmDir.cross(vPalmNormal).normalized();

    const float fThumbDist = hand.fingers()[Finger::TYPE_THUMB].bone(Bone::TYPE_METACARPAL).prevJoint().distanceTo(hand.palmPosition());
    const Vector vWrist = vPalm - fThumbDist*(vPalmDir*0.90f + (hand.isLeft() ? -1.0f : 1.0f)*vPalmSide*0.38f);

    FingerList fingers = hand.fingers();

    float fRadius = 0.0f;
    Vector vCurBoxBase;
    Vector vLastBoxBase = vWrist;

    for (int i = 0, ei = fingers.count(); i < ei; i++) {
      const Finger& finger = fingers[i];
      fRadius = finger.width() * 0.5f;

      // draw individual fingers
      for (int j = Bone::TYPE_METACARPAL; j <= Bone::TYPE_DISTAL; j++) {
        const Bone& bone = finger.bone(static_cast<Bone::Type>(j));

        // don't draw metacarpal, a box around the metacarpals is draw instead.
        if (j == Bone::TYPE_METACARPAL) {
          // cache the top of the metacarpal for the next step in metacarpal box
          vCurBoxBase = bone.nextJoint();
        } else {
          glColor4fv(vBoneColor);
          drawCylinder(kStyle_Solid, bone.prevJoint(), bone.nextJoint(), kfBoneRadiusScale*fRadius);
          glColor4fv(vJointColor);
          drawSphere(kStyle_Solid, bone.nextJoint(), kfJointRadiusScale*fRadius);
        }
      }

      // draw segment of metacarpal box
      glColor4fv(vBoneColor);
      drawCylinder(kStyle_Solid, vCurBoxBase, vLastBoxBase, kfBoneRadiusScale*fRadius);
      glColor4fv(vJointColor);
      drawSphere(kStyle_Solid, vCurBoxBase, kfJointRadiusScale*fRadius);
      vLastBoxBase = vCurBoxBase;
    }

    // close the metacarpal box
    fRadius = fingers[Finger::TYPE_THUMB].width() * 0.5f;
    vCurBoxBase = vWrist;
    glColor4fv(vBoneColor);
    drawCylinder(kStyle_Solid, vCurBoxBase, vLastBoxBase, kfBoneRadiusScale*fRadius);
    glColor4fv(vJointColor);
    drawSphere(kStyle_Solid, vCurBoxBase, kfJointRadiusScale*fRadius);

    // draw palm position
    glColor4fv(vJointColor);
    drawSphere(kStyle_Solid, vPalm, kfPalmRadiusScale*fRadius);
  }

//GLCamera methods

void CameraGL::SetupGLProjection() const
{
  ResetGLProjection();
  gluPerspective( GetVerticalFOVDegrees(), GetAspectRatio(), GetNearClip(), GetFarClip() );
}

void CameraGL::SetupGLView() const
{
  ResetGLView();
  glMultMatrixf( GetView().toArray4x4() );
}

} // namespace LeapUtilGL
