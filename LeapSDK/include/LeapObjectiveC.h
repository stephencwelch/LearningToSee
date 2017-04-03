/******************************************************************************\
* Copyright (C) 2012-2013 Leap Motion, Inc. All rights reserved.               *
* Leap Motion proprietary and confidential. Not for distribution.              *
* Use subject to the terms of the Leap Motion SDK Agreement available at       *
* https://developer.leapmotion.com/sdk_agreement, or another agreement         *
* between Leap Motion and you, your company or other organization.             *
\******************************************************************************/

#import <Foundation/Foundation.h>

/*************************************************************************
This wrapper works by doing a deep copy of the bulk of the hand and finger
hierarchy as soon as you the user requests `[controller frame]`. This is
enables us to set up the appropriate linkage between LeapHand and
LeapPointable ObjectiveC objects.

The motions API forced our hand to move the Frame and Hand objects towards
thin wrappers. Each now contains a pointer to its corresponding C++ object.
The screen API brought Pointable objects to wrap and keep around a C++
Leap::Pointable object as well.

Because the wrapped C++ object is kept around, attributes such as position
and velocity now have their ObjectiveC objects created lazily.

Major Leap API features supported in this wrapper today:
* Obtaining data through both polling (LeapController only) as well as
  through callbacks
* Waiting for single-threaded event callbacks through NSNotification objects
  (LeapListener), in addition to ObjectiveC delegates (LeapDelegate)
* Getting lists of hands, fingers, tools, or pointables from a frame or hand
  (e.g. `[frame hands]`, `[frame tools]`, or `[hand fingers]`)
* Getting hands, fingers, tools, or pointables a frame or hand by ID
  (e.g. `[frame hand:ID]`, `[frame tool:ID]`, or `[hand finger:ID]`)
* Querying back up the hierarchy, e.g. `[finger hand]` or `[hand frame]`
* Various hand/finger/tool properties: direction, palmNormal, sphereRadius,
  and more
* LeapVector math helper functions: pitch, roll, yaw, vector add, scalar
  multiply, dot product, cross product, LeapMatrix, and more
* Motions (translation/rotation/scale) from LeapHand and LeapFrame classes
* LeapGesture events, including swipe, circle, screen tap, and key tap
* LeapScreen location/orientation/size as well as intersection/projection
* LeapConfig class for configuring gesture recognition parameters

Notes:
* Class names are prefixed by Leap, although LM and LPM were considered.
  Users may change the prefix locally, for example:
    sed -i '.bak' 's/Leap\([A-NP-Z]\)/LPM\1/g' LeapObjectiveC.*
    # above regexp matches LeapController, LeapVector, not LeapObjectiveC
* Requires XCode 4.2+, relies on Automatic Reference Counting (ARC),
  minimum target OS X 10.7
* Contributions are welcome. Contact us via https://developer.leapmotion.com
*************************************************************************/

//////////////////////////////////////////////////////////////////////////
//VECTOR
/**
 * The LeapVector class represents a three-component mathematical vector or point
 * such as a direction or position in three-dimensional space.
 *
 * The Leap software employs a right-handed Cartesian coordinate system.
 * Values given are in units of real-world millimeters. The origin is centered
 * at the center of the Leap device. The x- and z-axes lie in the horizontal
 * plane, with the x-axis running parallel to the long edge of the device.
 * The y-axis is vertical, with positive values increasing upwards (in contrast
 * to the downward orientation of most computer graphics coordinate systems).
 * The z-axis has positive values increasing away from the computer screen.
 *
 * <img src="../docs/images/Leap_Axes.png"/>
 *
 * @available Since 1.0
 */
@interface LeapVector : NSObject

/**
 * Creates a new LeapVector with the specified component values.
 *
 * @example Vector_Constructor_1.txt
 *
 * @param x The horizontal component.
 * @param y The vertical component.
 * @param z The depth component.
 * @available Since 1.0
 */
- (id)initWithX:(float)x y:(float)y z:(float)z;
/**
 * Copies the specified LeapVector.
 *
 * @example Vector_Constructor_2.txt
 *
 * @param vector The LeapVector to copy.
 * @available Since 1.0
 */
- (id)initWithVector:(const LeapVector *)vector;
- (NSString *)description;
/**
 * The magnitude, or length, of this vector.
 *
 * @example Vector_Magnitude.txt
 *
 * The magnitude is the L2 norm, or Euclidean distance between the origin and
 * the point represented by the (x, y, z) components of this LeapVector object.
 *
 * @returns The length of this vector.
 * @available Since 1.0
 */
@property (nonatomic, getter = magnitude, readonly)float magnitude;
- (float)magnitude;
/**
 * The square of the magnitude, or length, of this vector.
 *
 * @example Vector_Magnitude_Squared.txt
 *
 * @returns The square of the length of this vector.
 * @available Since 1.0
 */
@property (nonatomic, getter = magnitudeSquared, readonly)float magnitudeSquared;
- (float)magnitudeSquared;
/**
 * The distance between the point represented by this LeapVector
 * object and a point represented by the specified LeapVector object.
 *
 * @example Vector_DistanceTo.txt
 *
 * @param vector A LeapVector object.
 * @returns The distance from this point to the specified point.
 * @available Since 1.0
 */
- (float)distanceTo:(const LeapVector *)vector;
/**
 *  The angle between this vector and the specified vector in radians.
 *
 * @example Vector_AngleTo.txt
 *
 * The angle is measured in the plane formed by the two vectors. The
 * angle returned is always the smaller of the two conjugate angles.
 * Thus `[A angleTo:B] == [B angleTo:A]` and is always a positive
 * value less than or equal to pi radians (180 degrees).
 *
 * If either vector has zero length, then this function returns zero.
 *
 * <img src="../docs/images/Math_AngleTo.png"/>
 *
 * @param vector A LeapVector object.
 * @returns The angle between this vector and the specified vector in radians.
 * @available Since 1.0
 */
- (float)angleTo:(const LeapVector *)vector;
/**
 *  The pitch angle in radians.
 *
 * @example Vector_Pitch.txt
 *
 * Pitch is the angle between the negative z-axis and the projection of
 * the vector onto the y-z plane. In other words, pitch represents rotation
 * around the x-axis.
 * If the vector points upward, the returned angle is between 0 and pi radians
 * (180 degrees); if it points downward, the angle is between 0 and -pi radians.
 *
 * <img src="../docs/images/Math_Pitch_Angle.png"/>
 *
 * @returns The angle of this vector above or below the horizon (x-z plane).
 * @available Since 1.0
 */
@property (nonatomic, getter = pitch, readonly)float pitch;
- (float)pitch;
/**
 *  The roll angle in radians.
 *
 * @example Vector_Roll.txt
 *
 * Roll is the angle between the y-axis and the projection of
 * the vector onto the x-y plane. In other words, roll represents rotation
 * around the z-axis. If the vector points to the left of the y-axis,
 * then the returned angle is between 0 and pi radians (180 degrees);
 * if it points to the right, the angle is between 0 and -pi radians.
 *
 * <img src="../docs/images/Math_Roll_Angle.png"/>
 *
 * Use this function to get roll angle of the plane to which this vector is a
 * normal. For example, if this vector represents the normal to the palm,
 * then this function returns the tilt or roll of the palm plane compared
 * to the horizontal (x-z) plane.
 *
 * @returns The angle of this vector to the right or left of the y-axis.
 * @available Since 1.0
 */
@property (nonatomic, getter = roll, readonly)float roll;
- (float)roll;
/**
 *  The yaw angle in radians.
 *
 * @example Vector_Yaw.txt
 *
 * Yaw is the angle between the negative z-axis and the projection of
 * the vector onto the x-z plane. In other words, yaw represents rotation
 * around the y-axis. If the vector points to the right of the negative z-axis,
 * then the returned angle is between 0 and pi radians (180 degrees);
 * if it points to the left, the angle is between 0 and -pi radians.
 *
 * <img src="../docs/images/Math_Yaw_Angle.png"/>
 *
 * @returns The angle of this vector to the right or left of the negative z-axis.
 * @available Since 1.0
 */
@property (nonatomic, getter = yaw, readonly)float yaw;
- (float)yaw;
/**
 * Adds two vectors.
 *
 * @example Vector_Plus.txt
 *
 * @param vector The LeapVector addend.
 * @returns The sum of the two LeapVectors.
 * @available Since 1.0
 */
- (LeapVector *)plus:(const LeapVector *)vector;
/**
 * Subtract a vector from this vector.
 *
 * @example Vector_Minus.txt
 *
 * @param vector the LeapVector subtrahend.
 * @returns the difference between the two LeapVectors.
 * @available Since 1.0
 */
- (LeapVector *)minus:(const LeapVector *)vector;
/**
 * Negate this vector.
 *
 * @example Vector_Negate.txt
 *
 * @returns The negation of this LeapVector.
 * @available Since 1.0
 */
- (LeapVector *)negate;
/**
 * Multiply this vector by a number.
 *
 * @example Vector_Times.txt
 *
 * @param scalar The scalar factor.
 * @returns The product of this LeapVector and a scalar.
 * @available Since 1.0
 */
- (LeapVector *)times:(float)scalar;
/**
 * Divide this vector by a number.
 *
 * @example Vector_Divide.txt
 *
 * @param scalar The scalar divisor;
 * @returns The dividend of this LeapVector divided by a scalar.
 * @available Since 1.0
 */
- (LeapVector *)divide:(float)scalar;
// not provided: unary assignment operators (plus_equals, minus_equals)
// user should emulate with above operators
/**
 * Checks LeapVector equality.
 *
 * @example Vector_Equals.txt
 *
 * Vectors are equal if each corresponding component is equal.
 * @param vector The LeapVector to compare.
 * @returns YES, if the LeapVectors are equal.
 * @available Since 1.0
 */
- (BOOL)equals:(const LeapVector *)vector;
// not provided: not_equals
// user should emulate with !v.equals(...)
/**
 *  The dot product of this vector with another vector.
 *
 * @example Vector_Dot.txt
 *
 * The dot product is the magnitude of the projection of this vector
 * onto the specified vector.
 *
 * <img src="../docs/images/Math_Dot.png"/>
 *
 * @param vector A LeapVector object.
 * @returns The dot product of this vector and the specified vector.
 * @available Since 1.0
 */
- (float)dot:(const LeapVector *)vector;
/**
 *  The cross product of this vector and the specified vector.
 *
 * @example Vector_Cross.txt
 *
 * The cross product is a vector orthogonal to both original vectors.
 * It has a magnitude equal to the area of a parallelogram having the
 * two vectors as sides. The direction of the returned vector is
 * determined by the right-hand rule. Thus `[A cross:B] ==
 * [[B negate] cross:A]`.
 *
 * <img src="../docs/images/Math_Cross.png"/>
 *
 * @param vector A LeapVector object.
 * @returns The cross product of this vector and the specified vector.
 * @available Since 1.0
 */
- (LeapVector *)cross:(const LeapVector *)vector;
/**
 *  A normalized copy of this vector.
 *
 * @example Vector_Normalized.txt
 *
 * A normalized vector has the same direction as the original vector,
 * but with a length of one.
 *
 * @returns A LeapVector object with a length of one, pointing in the same
 * direction as this Vector object.
 * @available Since 1.0
 */
@property (nonatomic, getter = normalized, readonly)LeapVector *normalized;
- (LeapVector *)normalized;
/**
 * Returns an NSArray object containing the vector components in the
 * order: x, y, z.
 *
 * @example Vector_ToNSArray.txt
 *
 * @available Since 1.0
 */
@property (nonatomic, getter = toNSArray, readonly)NSArray *toNSArray;
- (NSArray *)toNSArray;
/**
 * Returns an NSMutableData object containing the vector components as
 * consecutive floating point values.
 *
 * @example Vector_ToFloatPointer.txt
 * @available Since 1.0
 */
@property (nonatomic, getter = toFloatPointer, readonly)NSMutableData *toFloatPointer;
- (NSMutableData *)toFloatPointer;
// not provided: toVector4Type
// no templates, and ObjectiveC does not have a common math vector type
/**
 * The zero vector: (0, 0, 0)
 *
 * @example Vector_Zero.txt
 * @available Since 1.0
 */
+ (LeapVector *)zero;
/**
 * The x-axis unit vector: (1, 0, 0).
 *
 * @example Vector_XAxis.txt
 * @available Since 1.0
 */
+ (LeapVector *)xAxis;
/**
 * The y-axis unit vector: (0, 1, 0).
 *
 * @example Vector_YAxis.txt
 * @available Since 1.0
 */
+ (LeapVector *)yAxis;
/**
 * The z-axis unit vector: (0, 0, 1).
 *
 * @example Vector_ZAxis.txt
 * @available Since 1.0
 */
+ (LeapVector *)zAxis;
/**
 * The unit vector pointing left along the negative x-axis: (-1, 0, 0).
 *
 * @example Vector_Left.txt
 * @available Since 1.0
 */
+ (LeapVector *)left;
/**
 * The unit vector pointing right along the positive x-axis: (1, 0, 0).
 *
 * @example Vector_Right.txt
 * @available Since 1.0
 */
+ (LeapVector *)right;
/**
 * The unit vector pointing down along the negative y-axis: (0, -1, 0).
 *
 * @example Vector_Down.txt
 * @available Since 1.0
 */
+ (LeapVector *)down;
/**
 * The unit vector pointing up along the positive y-axis: (0, 1, 0).
 *
 * @example Vector_Up.txt
 * @available Since 1.0
 */
+ (LeapVector *)up;
/**
 * The unit vector pointing forward along the negative z-axis: (0, 0, -1).
 *
 * @example Vector_Forward.txt
 * @available Since 1.0
 */
+ (LeapVector *)forward;
/**
 * The unit vector pointing backward along the positive z-axis: (0, 0, 1).
 *
 * @example Vector_Backward.txt
 * @available Since 1.0
 */
+ (LeapVector *)backward;

/**
 * The horizontal component.
 * @available Since 1.0
 */
@property (nonatomic, assign, readwrite)float x;
/**
 * The vertical component.
 * @available Since 1.0
 */
@property (nonatomic, assign, readwrite)float y;
/**
 * The depth component.
 * @available Since 1.0
 */
@property (nonatomic, assign, readwrite)float z;

@end

//////////////////////////////////////////////////////////////////////////
//MATRIX
/**
 *  The LeapMatrix class represents a transformation matrix.
 *
 * To use this class to transform a LeapVector, construct a matrix containing the
 * desired transformation and then use the [LeapMatrix transformPoint:] or
 * [LeapMatrix transformDirection:] functions to apply the transform.
 *
 * Transforms can be combined by multiplying two or more transform matrices using
 * the [LeapMatrix times:] function.
 * @available Since 1.0
 */
@interface LeapMatrix : NSObject

/**
 *  Constructs a transformation matrix from the specified basis and translation vectors.
 *
 * @example Matrix_Constructor_1.txt
 *
 * @param xBasis A LeapVector specifying rotation and scale factors for the x-axis.
 * @param yBasis A LeapVector specifying rotation and scale factors for the y-axis.
 * @param zBasis A LeapVector specifying rotation and scale factors for the z-axis.
 * @param origin A LeapVector specifying translation factors on all three axes.
 * @available Since 1.0
 */
- (id)initWithXBasis:(const LeapVector *)xBasis yBasis:(const LeapVector *)yBasis zBasis:(const LeapVector *)zBasis origin:(const LeapVector *)origin;
/**
 * Constructs a copy of the specified Matrix object.
 *
 * @example Matrix_Constructor_2.txt
 *
 * @param matrix the LeapMatrix to copy.
 * @available Since 1.0
 */
- (id)initWithMatrix:(LeapMatrix *)matrix;
/**
 *  Constructs a transformation matrix specifying a rotation around the specified vector.
 *
 * @example Matrix_Constructor_3.txt
 *
 * @param axis A LeapVector specifying the axis of rotation.
 * @param angleRadians The amount of rotation in radians.
 * @available Since 1.0
 */
- (id)initWithAxis:(const LeapVector *)axis angleRadians:(float)angleRadians;
/**
 *  Constructs a transformation matrix specifying a rotation around the specified vector
 * and a translation by the specified vector.
 *
 * @example Matrix_Constructor_4.txt
 *
 * @param axis A LeapVector specifying the axis of rotation.
 * @param angleRadians The angle of rotation in radians.
 * @param translation A LeapVector representing the translation part of the transform.
 * @available Since 1.0
 */
- (id)initWithAxis:(const LeapVector *)axis angleRadians:(float)angleRadians translation:(const LeapVector *)translation;
- (NSString *)description;
// not provided: setRotation
// This was mainly an internal helper function for the above constructors
/**
 *  Transforms a vector with this matrix by transforming its rotation,
 * scale, and translation.
 *
 * Translation is applied after rotation and scale.
 *
 * @example Matrix_TransformPoint.txt
 *
 * @param point A LeapVector representing the 3D position to transform.
 * @returns A new LeapVector representing the transformed original.
 * @available Since 1.0
 */
- (LeapVector *)transformPoint:(const LeapVector *)point;
/**
 *  Transforms a vector with this matrix by transforming its rotation and
 * scale only.
 *
 * @example Matrix_TransformDirection.txt
 *
 * @param direction The LeapVector to transform.
 * @returns A new LeapVector representing the transformed original.
 * @available Since 1.0
 */
- (LeapVector *)transformDirection:(const LeapVector *)direction;
/**
 *  Performs a matrix inverse if the matrix consists entirely of rigid
 * transformations (translations and rotations).  If the matrix is not rigid,
 * this operation will not represent an inverse.
 *
 * @example Matrix_rigidInverse.txt
 *
 * Note that all matricies that are directly returned by the API are rigid.
 *
 * @returns The rigid inverse of the matrix.
 * @available Since 1.0
 */
@property (nonatomic, getter = rigidInverse, readonly)LeapMatrix *rigidInverse;
- (LeapMatrix *)rigidInverse;
/**
 *  Multiply transform matrices.
 *
 * @example Matrix_Times.txt
 *
 * Combines two transformations into a single equivalent transformation.
 *
 * @param other A LeapMatrix to multiply on the right hand side.
 * @returns A new LeapMatrix representing the transformation equivalent to
 * applying the other transformation followed by this transformation.
 * @available Since 1.0
 */
- (LeapMatrix *)times:(const LeapMatrix *) other;
// not provided: unary assignment operator times_equals
/**
 * Compare LeapMatrix equality component-wise.
 *
 * @example Matrix_Equals.txt
 *
 * @param other The LeapMatrix object to compare.
 * @return YES, if the corresponding elements in the two matrices are equal.
 * @available Since 1.0
 */
- (BOOL)equals:(const LeapMatrix *) other;
// not provided: not_equals
/**
 *  Converts a LeapMatrix object to a 9-element NSArray object.
 *
 * The elements of the matrix are inserted into the array in row-major order.
 *
 * @example Matrix_ToNSArray_1.txt
 *
 * Translation factors are discarded.
 * @available Since 1.0
 */
@property (nonatomic, getter = toNSArray3x3, readonly)NSMutableArray *toNSArray3x3;
- (NSMutableArray *)toNSArray3x3;
/**
 *  Converts a LeapMatrix object to a 16-element NSArray object.
 *
 * The elements of the matrix are inserted into the array in row-major order.
 *
 * @example Matrix_ToNSArray_2.txt
 *
 * @available Since 1.0
 */
@property (nonatomic, getter = toNSArray4x4, readonly)NSMutableArray *toNSArray4x4;
- (NSMutableArray *)toNSArray4x4;
/**
 *  Returns the identity matrix specifying no translation, rotation, and scale.
 *
 * @example Matrix_Identity.txt
 *
 * @returns The identity matrix.
 * @available Since 1.0
 */
+ (LeapMatrix *)identity;

/**
 * The rotation and scale factors for the x-axis.
 *
 * @example Matrix_xBasis.txt
 *
 * @available Since 1.0
 */
@property (nonatomic, strong, readwrite)LeapVector *xBasis;
/**
 * The rotation and scale factors for the y-axis.
 *
 * @example Matrix_yBasis.txt
 *
 * @available Since 1.0
 */
@property (nonatomic, strong, readwrite)LeapVector *yBasis;
/**
 * The rotation and scale factors for the z-axis.
 *
 * @example Matrix_zBasis.txt
 *
 * @available Since 1.0
 */
@property (nonatomic, strong, readwrite)LeapVector *zBasis;
/**
 * The translation factors for all three axes.
 *
 * @example Matrix_origin.txt
 *
 * @available Since 1.0
 */
@property (nonatomic, strong, readwrite)LeapVector *origin;

@end

//////////////////////////////////////////////////////////////////////////
//CONSTANTS
/**
 * The constant pi as a single precision floating point number.
 * @available Since 1.0
 */
extern const float LEAP_PI;
/**
 * The constant ratio to convert an angle measure from degrees to radians.
 * Multiply a value in degrees by this constant to convert to radians.
 * @available Since 1.0
 */
extern const float LEAP_DEG_TO_RAD;
/**
 * The constant ratio to convert an angle measure from radians to degrees.
 * Multiply a value in radians by this constant to convert to degrees.
 * @available Since 1.0
 */
extern const float LEAP_RAD_TO_DEG;

/**
 * The supported types of gestures.
 * @available Since 1.0
 */
typedef enum LeapGestureType {
    LEAP_GESTURE_TYPE_INVALID = -1, /**< An invalid type. */
    LEAP_GESTURE_TYPE_SWIPE = 1, /**< A straight line movement by the hand with fingers extended. */
    LEAP_GESTURE_TYPE_CIRCLE = 4, /**< A circular movement by a finger. */
    LEAP_GESTURE_TYPE_SCREEN_TAP = 5, /**< A forward tapping movement by a finger. */
    LEAP_GESTURE_TYPE_KEY_TAP = 6, /**< A downward tapping movement by a finger. */
} LeapGestureType;

/**
 * The possible gesture states.
 */
typedef enum LeapGestureState {
    LEAP_GESTURE_STATE_INVALID = -1, /**< An invalid state */
    LEAP_GESTURE_STATE_START = 1, /**< The gesture is starting. Just enough has happened to recognize it. */
    LEAP_GESTURE_STATE_UPDATE = 2, /**< The gesture is in progress. (Note: not all gestures have updates). */
    LEAP_GESTURE_STATE_STOP = 3, /**< The gesture has completed or stopped. */
} LeapGestureState;

/**
 * The supported controller policies.
 * @available Since 1.0
 */
typedef enum LeapPolicyFlag {
    LEAP_POLICY_DEFAULT = 0,                  /**< The default policy. Since 1.0*/
    LEAP_POLICY_BACKGROUND_FRAMES = (1 << 0), /**< Receive background frames. Since 1.0*/
    LEAP_POLICY_IMAGES = (1 << 1),            /**< Receive raw images from sensor cameras. Since 2.0.5*/
    LEAP_POLICY_OPTIMIZE_HMD = (1 << 2),      /**< Optimize the tracking for head-mounted device. Since 2.1.3*/
} LeapPolicyFlag;

/**
 * Defines the values for reporting the state of a Pointable object in relation to
 * an adaptive touch plane.
 * @available Since 1.0
 */
typedef enum LeapPointableZone {
    LEAP_POINTABLE_ZONE_NONE       = 0,  /**< The Pointable object is too far from
                                          the plane to be considered hovering or touching.*/
    LEAP_POINTABLE_ZONE_HOVERING   = 1,   /**< The Pointable object is close to, but
                                           not touching the plane.*/
    LEAP_POINTABLE_ZONE_TOUCHING   = 2,  /**< The Pointable has penetrated the plane. */
} LeapPointableZone;

/**
 * Enumerates the possible image formats.
 *
 * The [LeapImage format] method returns an item from the LEAP_IMAGE_FORMAT_TYPE enumeration.
 * @available Since 2.2.0
 */
typedef enum LeapImageFormatType {
    LEAP_IMAGE_FORMAT_TYPE_INFRARED = 0
} LeapImageFormatType;

/**
 * Enumerates the joints of a finger.
 *
 * The joints along the finger are indexed from 0 to 3 (tip to knuckle). The same
 * joint identifiers are used for the thumb, even though the thumb has one less
 * phalanx bone than the other digits. This puts the base joint (JOINT_MCP) at the
 * base of thumb's metacarpal bone.
 *
 * Pass a member of this enumeration to [LeapPointable jointPosition] to get the
 * physical position of that joint.
 *
 * Note: The term "joint" is applied loosely here and the set of joints includes the
 * finger tip even though it is not an anatomical joint.
 *
 * @available Since 2.0
 */
typedef enum LeapFingerJoint {
    /**
     * The metacarpophalangeal joint, or knuckle, of the finger.
     *
     * The metacarpophalangeal joint is located at the base of a finger between
     * the metacarpal bone and the first phalanx. The common name for this joint is
     * the knuckle.
     *
     * On a thumb, which has one less phalanx than a finger, this joint index
     * identifies the thumb joint near the base of the hand, between the carpal
     * and metacarpal bones.
     * @available Since 2.0
     */
    LEAP_FINGER_JOINT_MCP = 0,
    /**
     * The proximal interphalangeal joint of the finger. This joint is the middle
     * joint of a finger.
     *
     * The proximal interphalangeal joint is located between the two finger segments
     * closest to the hand (the proximal and the intermediate phalanges). On a thumb,
     * which lacks an intermediate phalanx, this joint index identifies the knuckle joint
     * between the proximal phalanx and the metacarpal bone.
     *
     * @available Since 2.0
     */
    LEAP_FINGER_JOINT_PIP = 1,
    /**
     * The distal interphalangeal joint of the finger.
     * This joint is closest to the tip.
     *
     * The distal interphalangeal joint is located between the most extreme segment
     * of the finger (the distal phalanx) and the middle segment (the intermediate
     * phalanx).
     *
     * @available Since 2.0
     */
    LEAP_FINGER_JOINT_DIP = 2,
    /**
     * The tip of the finger.
     * @available Since 2.0
     */
    LEAP_FINGER_JOINT_TIP = 3
} LeapFingerJoint;

/**
 * Enumerates the names of the fingers.
 *
 * Members of this enumeration are returned by [LeapFinger type] to identify a
 * Finger object.
 * @available Since 2.0
 */
typedef enum LeapFingerType {
    LEAP_FINGER_TYPE_THUMB  = 0, /**< The thumb */
    LEAP_FINGER_TYPE_INDEX  = 1, /**< The index or forefinger */
    LEAP_FINGER_TYPE_MIDDLE = 2, /**< The middle finger */
    LEAP_FINGER_TYPE_RING   = 3, /**< The ring finger */
    LEAP_FINGER_TYPE_PINKY  = 4  /**< The pinky or little finger */
} LeapFingerType;

//////////////////////////////////////////////////////////////////////////
//POINTABLE
@class LeapFrame;
@class LeapHand;
@class LeapInteractionBox;

/**
 * The LeapPointable class reports the physical characteristics of a detected finger or tool.
 *
 * Both fingers and tools are classified as LeapPointable objects. Use the
 * [LeapPointable isFinger] function to determine whether a pointable object
 * represents a finger. Use the [LeapPointable isTool] function to determine
 * whether a pointable object represents a tool. The Leap classifies a detected
 * entity as a tool when it is thinner, straighter, and longer than a typical finger.
 *
 * To provide touch emulation, the Leap Motion software associates a floating touch
 * plane that adapts to the user's finger movement and hand posture. The Leap Motion
 * interprets purposeful movements toward this plane as potential touch points. The
 * logic used by the LeapPointable class is the same as that used by the Leap Motion
 * software for OS-level touch and mouse input emulation. The LeapPointable class reports
 * touch state with the touchZone and touchDistance properties.
 *
 * Note that LeapPointable objects can be invalid, which means that they do not contain
 * valid tracking data and do not correspond to a physical entity. Invalid LeapPointable
 * objects can be the result of asking for a pointable object using an ID from an
 * earlier frame when no pointable objects with that ID exist in the current frame.
 * A pointable object created from the LeapPointable constructor is also invalid.
 * Test for validity with the [LeapPointable isValid] function.
 * @available Since 1.0
 */
@interface LeapPointable : NSObject

- (NSString *)description;
/**
 * A unique ID assigned to this LeapPointable object, whose value remains the
 * same across consecutive frames while the tracked finger or tool remains
 * visible. If tracking is lost, the
 * Leap may assign a new ID when it detects the entity in a future frame.
 *
 * @example Pointable_id.txt
 *
 * Use the ID value with the [LeapFrame pointable:] function to find this
 * LeapPointable object in future frames.
 *
 * @example Pointable_get_id.txt
 *
 * @returns The ID assigned to this LeapPointable object.
 * @available Since 1.0
 */
@property (nonatomic, getter = id, readonly)int32_t id;
- (int32_t)id;
/**
 * The tip position in millimeters from the Leap origin.
 *
 * @example Pointable_tipPosition.txt
 *
 * @returns The LeapVector containing the coordinates of the tip position.
 * @available Since 1.0
 */
@property (nonatomic, getter = tipPosition, readonly)LeapVector *tipPosition;
- (LeapVector *)tipPosition;
/**
 * The rate of change of the tip position in millimeters/second.
 *
 * @example Pointable_tipVelocity.txt
 *
 * @returns The LeapVector containing the coordinates of the tip velocity.
 * @available Since 1.0
 */
@property (nonatomic, getter = tipVelocity, readonly)LeapVector *tipVelocity;
- (LeapVector *)tipVelocity;
/**
 * The direction in which this finger or tool is pointing.
 *
 * @example Pointable_direction.txt
 *
 * The direction is expressed as a unit vector pointing in the same
 * direction as the tip.
 *
 * <img src="../docs/images/Leap_Finger_Model.png"/>
 *
 * @returns The LeapVector pointing in the same direction as the tip of this
 * LeapPointable object.
 * @available Since 1.0
 */
@property (nonatomic, getter = direction, readonly)LeapVector *direction;
- (LeapVector *)direction;
/**
 * The estimated width of the finger or tool in millimeters.
 *
 * The reported width is the average width of the visible portion of the
 * finger or tool. If the width isn't known,
 * then a value of 0 is returned.
 *
 * @example Pointable_width.txt
 *
 * @returns The estimated width of this LeapPointable object.
 * @available Since 1.0
 */
@property (nonatomic, getter = width, readonly)float width;
- (float)width;
/**
 * The estimated length of the finger or tool in millimeters.
 *
 * The reported length is the visible length of the finger or tool.
 * If the length isn't known, then a value of 0 is returned.
 *
 * @example Pointable_length.txt
 *
 * @returns The estimated length of this LeapPointable object.
 * @available Since 1.0
 */
@property (nonatomic, getter = length, readonly)float length;
- (float)length;
/**
 * Whether or not the LeapPointable is classified a finger.
 *
 * @example Pointable_isFinger.txt
 *
 * @returns YES, if this LeapPointable is classified as a LeapFinger.
 * @available Since 1.0
 */
@property (nonatomic, getter = isFinger, readonly)BOOL isFinger;
- (BOOL)isFinger;
/**
 * Whether or not the LeapPointable is classified to be a tool.
 *
 * @example Pointable_isTool.txt
 *
 * @returns YES, if this LeapPointable is classified as a LeapTool.
 * @available Since 1.0
 */
@property (nonatomic, getter = isTool, readonly)BOOL isTool;
- (BOOL)isTool;
/**
 * Whether or not this Pointable is in an extended posture.
 *
 * @example Pointable_isExtended.txt
 *
 * A finger is considered extended if it is extended straight from the hand as if
 * pointing. A finger is not extended when it is bent down and curled towards the
 * palm.  Tools are always extended.
 *
 * @returns True, if the pointable is extended.
 * @available Since 2.0
 */
@property (nonatomic, getter = isExtended, readonly)BOOL isExtended;
- (BOOL)isExtended;
/**
 * Reports whether this is a valid LeapPointable object.
 *
 * @example Pointable_isValid.txt
 *
 * @returns YES, if this LeapPointable object contains valid tracking data.
 * @available Since 1.0
 */
@property (nonatomic, getter = isValid, readonly)BOOL isValid;
- (BOOL)isValid;
/**
 * The current touch zone of this LeapPointable object.
 *
 * @example Pointable_touchZone.txt
 *
 * The Leap Motion software computes the touch zone based on a floating touch
 * plane that adapts to the user's finger movement and hand posture. The Leap
 * Motion interprets purposeful movements toward this plane as potential touch
 * points. When a LeapPointable moves close to the adaptive touch plane, it enters the
 * "hovering" zone. When a LeapPointable reaches or passes through the plane, it enters
 * the "touching" zone.
 *
 * The possible states are present in the Zone enum:
 *
 * **LEAP_POINTABLE_ZONE_NONE** -- The LeapPointable is outside the hovering zone.
 *
 * **LEAP_POINTABLE_ZONE_HOVERING** -- The LeapPointable is close to, but not touching the touch plane.
 *
 * **LEAP_POINTABLE_ZONE_TOUCHING** -- The LeapPointable has penetrated the touch plane.
 *
 *  <img src="../docs/images/Leap_Touch_Plane.png"/>
 *
 * The touchDistance property provides a normalized indication of the distance to
 * the touch plane when the LeapPointable is in the hovering or touching zones.
 * @available Since 1.0
 */
@property (nonatomic, getter = touchZone, readonly)LeapPointableZone touchZone;
- (LeapPointableZone)touchZone;
/**
 * A value proportional to the distance between this LeapPointable object and the
 * adaptive touch plane.
 *
 * @example Pointable_touchDistance.txt
 *
 * The touch distance is a value in the range [-1, 1]. The value 1.0 indicates the
 * LeapPointable is at the far edge of the hovering zone. The value 0 indicates the
 * LeapPointable is just entering the touching zone. A value of -1.0 indicates the
 * LeapPointable is firmly within the touching zone. Values in between are
 * proportional to the distance from the plane. Thus, the touchDistance of 0.5
 * indicates that the LeapPointable is halfway into the hovering zone.
 *
 * You can use the touchDistance value to modulate visual feedback given to the
 * user as their fingers close in on a touch target, such as a button.
 * @available Since 1.0
 */
@property (nonatomic, getter = touchDistance, readonly)float touchDistance;
- (float)touchDistance;
/**
 * The stabilized tip position of this LeapPointable.
 *
 * @example Pointable_stabilizedTipPosition.txt
 *
 * Smoothing and stabilization is performed in order to make
 * this value more suitable for interaction with 2D content.
 * @available Since 1.0
 */
@property (nonatomic, getter = stabilizedTipPosition, readonly)LeapVector *stabilizedTipPosition;
- (LeapVector *)stabilizedTipPosition;
/**
 * The duration of time this Pointable has been visible to the Leap Motion Controller.
 *
 * @example Pointable_timeVisible.txt
 *
 * @returns The duration (in seconds) that this Pointable has been tracked.
 * @available Since 1.0
 */
@property (nonatomic, getter = timeVisible, readonly)float timeVisible;
- (float)timeVisible;
/**
 * The LeapFrame associated with this LeapPointable object.
 *
 * @example Pointable_frame.txt
 *
 * This property is a weak reference to the LeapFrame object so it is only valid
 * during the lifetime of the LeapFrame object -- while the LeapFrame object is in
 * the history buffer or while your application maintains its own reference.
 *
 * @returns The associated LeapFrame object, if available; otherwise,
 * an invalid LeapFrame object is returned.
 * @available Since 1.0
 */
@property (nonatomic, weak, getter = frame, readonly)LeapFrame *frame;
- (LeapFrame *)frame;
/**
 * The LeapHand associated with a finger.
 *
 * @example Pointable_hand.txt
 *
 * This property is a weak reference to the LeapHand object so it is only valid
 * during the lifetime of that LeapHand object -- in other words, while the
 * parent LeapFrame object is valid or while your application maintains its own
 * reference.
 *
 * As of version 2, tools are not associated with hands, so this property always
 * returns an invalid LeapHand object for tools.
 *
 * @returns The associated LeapHand object, if available; otherwise,
 * an invalid LeapHand object is returned.
 * @available Since 1.0
 */
@property (nonatomic, weak, getter = hand, readonly)LeapHand *hand;
- (LeapHand *)hand;
/**
 * Returns an invalid LeapPointable object.
 *
 * @example Pointable_invalid.txt
 *
 * @returns The invalid LeapPointable instance.
 * @available Since 1.0
 */
+ (LeapPointable *)invalid;

@end

//////////////////////////////////////////////////////////////////////////
//ARM
/**
 * The LeapArm class represents the forearm attached to a tracked hand.
 *
 * @available Since 2.1.2
 */
@interface LeapArm : NSObject

/**
 * The orthonormal basis vectors for the LeapArm bone as a Matrix.
 *
 * Basis vectors specify the orientation of a bone.
 *
 * **xBasis** Perpendicular to the longitudinal axis of the
 *   bone; exits the arm laterally through the sides of the wrist.
 *
 * **yBasis or up vector** Perpendicular to the longitudinal
 *   axis of the bone; exits the top and bottom of the arm. More positive
 *   in the upward direction.
 *
 * **zBasis** Aligned with the longitudinal axis of the arm bone.
 *   More positive toward the wrist.
 *
 * \include Arm_basis.txt
 *
 * The bases provided for the right arm use the right-hand rule; those for
 * the left arm use the left-hand rule. Thus, the positive direction of the
 * x-basis is to the right for the right arm and to the left for the left
 * arm. You can change from right-hand to left-hand rule by multiplying the
 * z basis vector by -1.
 *
 * Note that converting the basis vectors directly into a quaternion
 * representation is not mathematically valid. If you use quaternions,
 * create them from the derived rotation matrix not directly from the bases.
 *
 * @returns The basis of the arm bone as a matrix.
 * @since 2.1.2
 */
@property (nonatomic, getter = basis, readonly)LeapMatrix *basis;
- (LeapMatrix *)basis;
/**
 * The center of the forearm.
 *
 * This location represents the midpoint of the arm between the wrist position
 * and the elbow position.
 * @since 2.1.2
 */
@property (nonatomic, getter = center, readonly)LeapVector *center;
- (LeapVector *)center;
/**
 * The normalized direction in which the arm is pointing (from elbow to wrist).
 *
 * @example Arm_direction.txt
 *
 * @since 2.1.2
 */
@property (nonatomic, getter = direction, readonly)LeapVector *direction;
- (LeapVector *)direction;
/**
 * Reports whether this is a valid LeapArm object.
 *
 * @example Arm_isValid.txt
 *
 * @since 2.1.2
 */
@property (nonatomic, getter = isValid, readonly)BOOL isValid;
- (BOOL)isValid;
/**
 * The position of the elbow.
 *
 * @example Arm_elbowPosition.txt
 *
 * If not in view, the elbow position is estimated based on typical human
 * anatomical proportions.
 * @since 2.1.2
 */
@property (nonatomic, getter = elbowPosition, readonly)LeapVector *elbowPosition;
- (LeapVector *)elbowPosition;
/**
 * The position of the wrist.
 *
 * \include Arm_wristPosition.txt
 *
 * Note that the wrist position is not collocated with the end of any bone in
 * the hand. There is a gap of a few centimeters since the carpal bones are
 * not included in the skeleton model.
 * @since 2.1.2
 */
@property (nonatomic, getter = wristPosition, readonly)LeapVector *wristPosition;
- (LeapVector *)wristPosition;
/**
 * The average width of the arm.
 *
 * \include Arm_width.txt
 * @since 2.1.2
 */
@property (nonatomic, getter = width, readonly)float width;
- (float)width;
/**
 * An invalid LeapArm object.
 *
 * You can use an invalid LeapArm object in comparisons testing
 * whether a given LeapArm instance is valid or invalid. (You can also use the
 * LeapArm.isValid property.)
 *
 * @example Arm_invalid.txt
 *
 * @returns The invalid Arm instance.
 * @since 2.1.2
 */
@property (nonatomic, getter = invalid, readonly)LeapArm *invalid;
+ (LeapArm *)invalid;

@end

/**
 * Enumerates the names of the bones.
 * @available Since 2.0
 */
typedef enum LeapBoneType {
    LEAP_BONE_TYPE_METACARPAL  = 0, /**< The metacarpal bone connecting to the wrist */
    LEAP_BONE_TYPE_PROXIMAL  = 1, /**< The first finger phalanx bone. */
    LEAP_BONE_TYPE_INTERMEDIATE = 2, /**< The middle finger phalanx bone */
    LEAP_BONE_TYPE_DISTAL  = 3  /**< The finger phalanx bone at the end of the finger */
} LeapBoneType;


//////////////////////////////////////////////////////////////////////////
//BONE
/**
 * The LeapBone class represents a tracked bone.
 *
 * All fingers contain 4 bones that make up the anatomy of the finger.
 * Get valid LeapBone objects from a Finger object.
 *
 * Bones are ordered from base to tip, indexed from 0 to 3.  Additionally, the
 * bone's LeapBoneType enum may be used to index a specific bone anatomically.
 *
 * The thumb does not have a base metacarpal bone and therefore contains a valid,
 * zero length bone at that location.
 *
 * Note that LeapBone objects can be invalid, which means that they do not contain
 * valid tracking data and do not correspond to a physical bone. Invalid LeapBone
 * objects can be the result of asking for a LeapBone object from an invalid finger,
 * indexing a bone out of range, or constructing a new bone.
 * Test for validity with the LeapBone.isValid property.
 * @available Since 2.0
 */
@interface LeapBone : NSObject

/**
 * The orientation of the bone as a basis matrix.
 *
 * The basis is defined as follows:
 *   xAxis: Clockwise rotation axis of the bone
 *   yAxis: Positive above the bone
 *   zAxis: Positive along the bone towards the wrist
 *
 * @example Bone_basis.txt
 *
 * Note: Since the left hand is a mirror of the right hand, left handed
 * bones will contain a left-handed basis.
 *
 * @returns The basis of the bone as a matrix.
 * @since 2.0
 */
@property (nonatomic, getter = basis, readonly)LeapMatrix *basis;
- (LeapMatrix *)basis;
/**
 * The midpoint in the center of the bone.
 *
 * @example Bone_center.txt
 *
 * @returns The midpoint in the center of the bone.
 * @since 2.0
 */
@property (nonatomic, getter = center, readonly)LeapVector *center;
- (LeapVector *)center;
/**
 * The normalized direction of the bone from base to tip.
 *
 * @example Bone_direction.txt
 *
 * @returns The normalized direction of the bone from base to tip.
 * @since 2.0
 */
@property (nonatomic, getter = direction, readonly)LeapVector *direction;
- (LeapVector *)direction;
/**
 * Reports whether this is a valid LeapBone object.
 *
 * @example Bone_isValid.txt
 *
 * @since 2.0
 */
@property (nonatomic, getter = isValid, readonly)BOOL isValid;
- (BOOL)isValid;
/**
 * The end of the bone closest from the wrist.
 *
 * @example Bone_prevJoint.txt
 *
 * @returns The LeapVector containing the coordinates of the previous joint position
 * (the joint closer to the wrist).
 * @since 2.0
 */
@property (nonatomic, getter = prevJoint, readonly)LeapVector *prevJoint;
- (LeapVector *)prevJoint;
/**
 * The end of the bone farthest from the wrist.
 *
 * @example Bone_nextJoint.txt
 *
 * @returns The LeapVector containing the coordinates of the next joint position
 * (the joint or bone tip closest to the end of the finger).
 * @since 2.0
 */
@property (nonatomic, getter = nextJoint, readonly)LeapVector *nextJoint;
- (LeapVector *)nextJoint;
/**
 * The estimated length of the bone in millimeters.
 *
 * @example Bone_length.txt
 *
 * @returns The length of the bone in millimeters.
 * @since 2.0
 */
@property (nonatomic, getter = length, readonly)float length;
- (float)length;
/**
 * The average width of the finger along this bone in millimeters.
 *
 * @example Bone_width.txt
 *
 * @returns The average width of the bone including the surrounding flesh in millimeters.
 * @since 2.0
 */
@property (nonatomic, getter = width, readonly)float width;
- (float)width;
/**
 * The name of this bone.
 *
 * @example Bone_type.txt
 *
 * @returns The anatomical type of this bone as a member of the LeapBoneType
 * enumeration.
 * @since 2.0
 */
@property (nonatomic, getter = type, readonly)LeapBoneType type;
- (LeapBoneType)type;
/**
 * An invalid Bone object.
 *
 * You can use an invalid LeapBone object in comparisons testing
 * whether a given LeapBone instance is valid or invalid. (You can also use the
 * LeapBone.isValid property.)
 *
 * @example Bone_invalid.txt
 *
 * @returns The invalid LeapBone instance.
 * @since 2.0
 */
@property (nonatomic, getter = invalid, readonly)LeapBone *invalid;
+ (LeapBone *)invalid;

@end
//////////////////////////////////////////////////////////////////////////
//FINGER
/**
 * The LeapFinger class represents a tracked finger.
 *
 * Fingers are pointable objects that the Leap has classified as a finger.
 * Get valid LeapFinger objects from a LeapFrame or a LeapHand object.
 *
 * Note that LeapFinger objects can be invalid, which means that they do not contain
 * valid tracking data and do not correspond to a physical finger. Invalid LeapFinger
 * objects can be the result of asking for a finger using an ID from an
 * earlier frame when no fingers with that ID exist in the current frame.
 * A LeapFinger object created from the LeapFinger constructor is also invalid.
 * Test for validity with the LeapFinger isValid function.
 * @available Since 1.0
 */
@interface LeapFinger : LeapPointable

/**
 * The position of the specified joint on this finger in millimeters from the
 * Leap Motion origin.
 *
 * @deprecated Use the Bone class prevJoint and nextJoint properties instead.
 *
 * @param jointIx An index value from the LeapFingerJoint enumeration identifying the
 * joint of interest.
 * @returns The LeapVector containing the coordinates of the joint position.
 * @available Since 2.0
 */
- (LeapVector *)jointPosition:(LeapFingerJoint)jointIx;
/**
 * The name of this finger.
 *
 * @example Finger_type.txt
 *
 * @returns The anatomical type of this finger as a member of the LeapFingerType
 * enumeration.
 * @available Since 2.0
 */
@property (nonatomic, getter = type, readonly)LeapFingerType type;
- (LeapFingerType)type;

/**
 * The bone of the specified anatomical type for this finger.
 *
 * Note that the thumb bone types match the types of the other fingers in the
 * Leap Motion model. A real thumb has one fewer bone than the other fingers,
 * so a zero-length bone is inserted for the thumb in the Leap Motion model.
 * To keep the bone indexes and names consistent between the thumb and other fingers,
 * the zero-length bone is added at the metacarpal position. However, in the standard
 * anatomical naming system, the missing thumb bone is the intermediate
 * phalanx. Thus, in the Leap Motion model, the anatomical metacarpal of the thumb is
 * labeled as the proximal phalanx and the proximal phalanx is labeled as the
 * intermediate phalanx.
 *
 * @example Finger_bone.txt
 *
 * @param type An index value from the LeapBoneType enumeration identifying the
 * bone of interest.
 * @returns The LeapBone object of the specified bone type.
 * @since 2.0
 */
- (LeapBone *)bone:(LeapBoneType)type;
@end

//////////////////////////////////////////////////////////////////////////
//TOOL
/**
 * The LeapTool class represents a tracked tool.
 *
 * Tools are pointable objects that the Leap has classified as a tool.
 * Tools are longer, thinner, and straighter than a typical finger.
 * Get valid LeapTool objects from a LeapFrame object.
 *
 * <img src="../docs/images/Leap_Tool.png"/>
 *
 * Note that LeapTool objects can be invalid, which means that they do not contain
 * valid tracking data and do not correspond to a physical tool. Invalid LeapTool
 * objects can be the result of asking for a tool object using an ID from an
 * earlier frame when no tools with that ID exist in the current frame.
 * A LeapTool object created from the LeapTool constructor is also invalid.
 * Test for validity with the LeapTool isValid function.
 * @available Since 1.0
 */
@interface LeapTool : LeapPointable
@end

//////////////////////////////////////////////////////////////////////////
//HAND
/**
 * The LeapHand class reports the physical characteristics of a detected hand.
 *
 * Hand tracking data includes a palm position and velocity; vectors for
 * the palm normal and direction to the fingers; properties of a sphere fit
 * to the hand; and lists of the attached fingers.
 *
 * Note that LeapHand objects can be invalid, which means that they do not contain
 * valid tracking data and do not correspond to a physical entity. Invalid LeapHand
 * objects can be the result of asking for a Hand object using an ID from an
 * earlier frame when no hand objects with that ID exist in the current frame.
 * A hand object created from the LeapHand constructor is also invalid.
 * Test for validity with the LeapHand isValid function.
 * @available Since 1.0
 */
@interface LeapHand : NSObject

- (NSString *)description;
/**
 * A unique ID assigned to this LeapHand object, whose value remains the same
 * across consecutive frames while the tracked hand remains visible.
 *
 * If tracking is lost (for example, when a hand is occluded by another hand
 * or when it is withdrawn from or reaches the edge of the Leap field of view),
 * the Leap may assign a new ID when it detects the hand in a future frame.
 *
 * @example Hand_id.txt
 *
 * Use the ID value with the [LeapFrame hand:] function to find this LeapHand object
 * in future frames.
 *
 * @example Hand_Get_ID.txt
 *
 * @returns The ID of this hand.
 * @available Since 1.0
 */
@property (nonatomic, getter = id, readonly)int32_t id;
- (int32_t)id;
/**
 * The list of LeapPointable objects (fingers) detected in this frame
 * that are associated with this hand, given in arbitrary order.
 *
 * @example Hand_pointables.txt
 *
 * The list will always contain 5 fingers even when some fingers are blocked
 * from view and when the user is missing digits.
 *
 * @returns An NSArray containing all LeapPointable objects associated with this hand.
 * @available Since 1.0
 */
@property (nonatomic, getter = pointables, readonly)NSArray *pointables;
- (NSArray *)pointables;
/**
 * The list of LeapFinger objects detected in this frame that are attached to
 * this hand, given in arbitrary order.
 *
 * @example Hand_fingers.txt
 *
 * The list can be empty if no fingers attached to this hand are detected.
 *
 * @returns An NSArray containing all LeapFinger objects attached to this hand.
 * @available Since 1.0
 */
@property (nonatomic, getter = fingers, readonly)NSArray *fingers;
- (NSArray *)fingers;
/*
 * The list of LeapTool objects detected in this frame that are held by this
 * hand, given in arbitrary order.
 *
 * @example Hand_tools.txt
 *
 * The list can be empty if no tools held by this hand are detected.
 *
 * @returns An NSArray containing all LeapTool objects held by this hand.
 * @available Since 1.0
 */
 /**
 * @deprecated 2.0
 */
@property (nonatomic, getter = tools, readonly)NSArray *tools;
- (NSArray *)tools;
/**
 * The LeapPointable object with the specified ID associated with this hand.
 *
 * Use this [LeapHand pointable:] function to retrieve a LeapPointable object
 * associated with this hand using an ID value obtained from a previous frame.
 * This function always returns a LeapPointable object, but if no finger
 * with the specified ID is present, an invalid LeapPointable object is returned.
 *
 * @example Hand_pointable.txt
 *
 * Note that ID values of fingers are based on the hand ID. Hand IDs persist
 * across frames, but only until tracking of that hand is lost. If tracking of
 * a hand is lost and subsequently
 * regained, the new LeapHand object representing that hand will have a different
 * ID than that representing the hand in an earlier frame. Thus the IDs for the
 * fingers on the hand will also change.
 *
 * @param pointableId The ID value of a LeapPointable object from a previous frame.
 * @returns The LeapPointable object with the matching ID if one exists for this
 * hand in this frame; otherwise, an invalid LeapPointable object is returned.
 * @available Since 1.0
 */
- (LeapPointable *)pointable:(int32_t)pointableId;
/**
 * The LeapFinger object with the specified ID attached to this hand.
 *
 * Use this [LeapHand finger:] function to retrieve a LeapFinger object attached to
 * this hand using an ID value obtained from a previous frame.
 * This function always returns a LeapFinger object, but if no finger
 * with the specified ID is present, an invalid LeapFinger object is returned.
 *
 * @example Hand_finger.txt
 *
 * Note that ID values persist across frames, but only until tracking of a
 * particular object is lost. If tracking of a finger is lost and subsequently
 * regained, the new LeapFinger object representing that finger may have a
 * different ID than that representing the finger in an earlier frame.
 *
 * @param fingerId The ID value of a LeapFinger object from a previous frame.
 * @returns The LeapFinger object with the matching ID if one exists for this
 * hand in this frame; otherwise, an invalid LeapFinger object is returned.
 * @available Since 1.0
 */
- (LeapFinger *)finger:(int32_t)fingerId;
/*
 * The LeapTool object with the specified ID held by this hand.
 *
 * Use this [LeapHand tool:] function to retrieve a LeapTool object held by
 * this hand using an ID value obtained from a previous frame.
 * This function always returns a LeapTool object, but if no tool
 * with the specified ID is present, an invalid LeapTool object is returned.
 *
 * @example Hand_tool.txt
 *
 * Note that ID values persist across frames, but only until tracking of a
 * particular object is lost. If tracking of a tool is lost and subsequently
 * regained, the new LeapTool object representing that tool may have a
 * different ID than that representing the tool in an earlier frame.
 *
 * @param toolId The ID value of a LeapTool object from a previous frame.
 * @returns The LeapTool object with the matching ID if one exists for this
 * hand in this frame; otherwise, an invalid LeapTool object is returned.
 * @available Since 1.0
 */
 /**
 * @deprecated 2.0
 */
- (LeapTool *)tool:(int32_t)toolId;
/**
 * The arm to which this hand is attached.
 *
 * If the arm is not completely in view, Arm attributes are estimated based on
 * the attributes of entities that are in view combined with typical human anatomy.
 *
 * \include Arm_get.txt
 *
 * @returns The Arm object for this hand.
 * @since 2.0.3
 */
@property (nonatomic, getter = arm, readonly)LeapArm *arm;
- (LeapArm *)arm;
/**
 * The center position of the palm in millimeters from the Leap origin.
 *
 * @example Hand_palmPosition.txt
 *
 * @returns The LeapVector representing the coordinates of the palm position.
 * @available Since 1.0
 */
@property (nonatomic, getter = palmPosition, readonly)LeapVector *palmPosition;
- (LeapVector *)palmPosition;
/**
 * The stabilized tip position of this Pointable.
 *
 * @example Hand_stabilizedPalmPosition.txt
 *
 * Smoothing and stabilization is performed in order to make
 * this value more suitable for interaction with 2D content.
 *
 * @returns A modified tip position of this Pointable object
 * with some additional smoothing and stabilization applied.
 * @available Since 1.0
 */
@property (nonatomic, getter = stabilizedPalmPosition, readonly)LeapVector *stabilizedPalmPosition;
 - (LeapVector *)stabilizedPalmPosition;
/**
 * The rate of change of the palm position in millimeters/second.
 *
 * @example Hand_palmVelocity.txt
 *
 * @returns The LeapVector representing the coordinates of the palm velocity.
 * @available Since 1.0
 */
@property (nonatomic, getter = palmVelocity, readonly)LeapVector *palmVelocity;
- (LeapVector *)palmVelocity;
/**
 * The normal vector to the palm. If your hand is flat, this vector will
 * point downward, or "out" of the front surface of your palm.
 *
 * <img src="../docs/images/Leap_Palm_Vectors.png"/>
 *
 * The direction is expressed as a unit vector pointing in the same
 * direction as the palm normal (that is, a vector orthogonal to the palm).
 *
 * @example Hand_palmNormal.txt
 *
 * @returns The LeapVector normal to the plane formed by the palm.
 * @available Since 1.0
 */
@property (nonatomic, getter = palmNormal, readonly)LeapVector *palmNormal;
- (LeapVector *)palmNormal;
/**
 * The direction from the palm position toward the fingers.
 *
 * @example Hand_direction.txt
 *
 * The direction is expressed as a unit vector pointing in the same
 * direction as the directed line from the palm position to the fingers.
 *
 * @returns The LeapVector pointing from the palm position toward the fingers.
 * @available Since 1.0
 */
@property (nonatomic, getter = direction, readonly)LeapVector *direction;
- (LeapVector *)direction;
/**
 * The center of a sphere fit to the curvature of this hand.
 *
 * @example Hand_sphereCenter.txt
 *
 * This sphere is placed roughly as if the hand were holding a ball.
 *
 * <img src="../docs/images/Leap_Hand_Ball.png"/>
 *
 * @returns The LeapVector representing the center position of the sphere.
 * @available Since 1.0
 */
@property (nonatomic, getter = sphereCenter, readonly)LeapVector *sphereCenter;
- (LeapVector *)sphereCenter;
/**
 * The radius of a sphere fit to the curvature of this hand.
 *
 * @example Hand_sphereRadius.txt
 *
 * This sphere is placed roughly as if the hand were holding a ball. Thus the
 * size of the sphere decreases as the fingers are curled into a fist.
 * @returns The radius of the sphere in millimeters.
 * @available Since 1.0
 */
@property (nonatomic, getter = sphereRadius, readonly)float sphereRadius;
- (float)sphereRadius;
/**
 * The holding strength of a pinch hand pose.
 *
 * @example Hand_pinchStrength.txt
 *
 * The strength is zero for an open hand, and blends to 1.0 when a pinching
 * hand pose is recognized. Pinching can be done between the thumb
 * and any other finger of the same hand.
 *
 * @returns A float value in the [0..1] range representing the holding strength
 * of the pinch pose.
 * @available Since 2.0
 */
@property (nonatomic, getter = pinchStrength, readonly)float pinchStrength;
- (float) pinchStrength;
/**
 * The strength of a grab hand pose.
 *
 * @example Hand_grabStrength.txt
 *
 * The strength is zero for an open hand, and blends to 1.0 when a grabbing hand
 * pose is recognized.
 *
 * @returns A float value in the [0..1] range representing the holding strength
 * of the pose.
 * @available Since 2.0
 */
@property (nonatomic, getter = grabStrength, readonly)float grabStrength;
- (float) grabStrength;
/**
 * Reports whether this is a valid LeapHand object.
 *
 * @example Hand_isValid.txt
 *
 * @returns YES, if this LeapHand object contains valid tracking data.
 * @available Since 1.0
 */
@property (nonatomic, getter = isValid, readonly)BOOL isValid;
- (BOOL)isValid;
/**
 * The LeapFrame associated with this Hand.
 *
 * @example Hand_frame.txt
 *
 * This property is a weak reference to the LeapFrame object so it is only valid
 * during the lifetime of the LeapFrame object -- while the LeapFrame object is in
 * the history buffer or while your application maintains its own reference.
 *
 * @returns The associated LeapFrame object, if available; otherwise,
 * an invalid LeapFrame object is returned.
 * @available Since 1.0
 */
@property (nonatomic, weak, getter = frame, readonly)LeapFrame *frame;
- (LeapFrame *)frame;
/**
 * The change of position of this hand between the current LeapFrame and
 * the specified LeapFrame.
 *
 * The returned translation vector provides the magnitude and direction of
 * the movement in millimeters.
 *
 * @example Hand_translation.txt
 *
 * If a corresponding LeapHand object is not found in sinceFrame, or if either
 * this frame or sinceFrame are invalid LeapFrame objects, then this method
 * returns a zero vector.
 *
 * @param sinceFrame The starting LeapFrame for computing the translation.
 * @returns A LeapVector representing the heuristically determined change in
 * hand position between the current frame and that specified in the
 * sinceFrame parameter.
 * @available Since 1.0
 */
- (LeapVector *)translation:(const LeapFrame *)sinceFrame;
/**
 * The estimated probability that the hand motion between the current
 * frame and the specified LeapFrame is intended to be a translating motion.
 *
 * @example Hand_translationProbablity.txt
 *
 * If a corresponding LeapHand object is not found in sinceFrame, or if either
 * this frame or sinceFrame are invalid LeapFrame objects, then this method
 * returns zero.
 *
 * @param sinceFrame The starting LeapFrame for computing the translation.
 * @returns A value between 0 and 1 representing the estimated probability
 * that the hand motion between the current frame and the specified frame
 * is intended to be a translating motion.
 * @available Since 1.0
 */
- (float)translationProbability:(const LeapFrame *)sinceFrame;
/**
 * The axis of rotation derived from the change in orientation of this
 * hand, and associated fingers, between the current LeapFrame
 * and the specified LeapFrame.
 *
 * The returned direction vector is normalized.
 *
 * @example Hand_rotationAxis.txt
 *
 * If a corresponding LeapHand object is not found in sinceFrame, or if either
 * this frame or sinceFrame are invalid LeapFrame objects, then this method
 * returns a zero vector.
 *
 * @param sinceFrame The starting LeapFrame for computing the relative rotation.
 * @returns A LeapVector containing the normalized direction vector representing the heuristically
 * determined axis of rotational change of the hand between the current
 * frame and that specified in the sinceFrame parameter.
 * @available Since 1.0
 */
- (LeapVector *)rotationAxis:(const LeapFrame *)sinceFrame;
/**
 * The angle of rotation around the rotation axis derived from the change
 * in orientation of this hand, and associated fingers,
 * between the current LeapFrame and the specified LeapFrame.
 *
 * The returned angle is expressed in radians measured clockwise around the
 * rotation axis (using the right-hand rule) between the start and end frames.
 * The value is always between 0 and pi radians (0 and 180 degrees).
 *
 * @example Hand_rotationAngle.txt
 *
 * If a corresponding LeapHand object is not found in sinceFrame, or if either
 * this frame or sinceFrame are invalid LeapFrame objects, then the angle of
 * rotation is zero.
 *
 * @param sinceFrame The starting LeapFrame for computing the relative rotation.
 * @returns A positive value representing the heuristically determined
 * rotational change of the hand between the current frame and that
 * specified in the sinceFrame parameter.
 * @available Since 1.0
 */
- (float)rotationAngle:(const LeapFrame *)sinceFrame;
/**
 * The angle of rotation around the specified axis derived from the change
 * in orientation of this hand, and associated fingers,
 * between the current LeapFrame and the specified LeapFrame.
 *
 * The returned angle is expressed in radians measured clockwise around the
 * rotation axis (using the right-hand rule) between the start and end frames.
 * The value is always between -pi and pi radians (-180 and 180 degrees).
 *
 * @example Hand_rotationAngle_axis.txt
 *
 * If a corresponding LeapHand object is not found in sinceFrame, or if either
 * this frame or sinceFrame are invalid LeapFrame objects, then the angle of
 * rotation is zero.
 *
 * @param sinceFrame The starting LeapFrame for computing the relative rotation.
 * @param axis A LeapVector representing the axis to measure rotation around.
 * @returns A value representing the heuristically determined rotational
 * change of the hand between the current frame and that specified in the
 * sinceFrame parameter around the specified axis.
 * @available Since 1.0
 */
- (float)rotationAngle:(const LeapFrame *)sinceFrame axis:(const LeapVector *)axis;
/**
 * The transform matrix expressing the rotation derived from the change
 * in orientation of this hand, and associated fingers,
 * between the current LeapFrame and the specified LeapFrame.
 *
 * @example Hand_rotationMatrix.txt
 *
 * If a corresponding LeapHand object is not found in sinceFrame, or if either
 * this frame or sinceFrame are invalid LeapFrame objects, then this method
 * returns an identity matrix.
 *
 * @param sinceFrame The starting LeapFrame for computing the relative rotation.
 * @returns A transformation LeapMatrix representing the heuristically determined
 * rotational change of the hand between the current frame and that specified
 * in the sinceFrame parameter.
 * @available Since 1.0
 */
- (LeapMatrix *)rotationMatrix:(const LeapFrame *)sinceFrame;
/**
 * The estimated probability that the hand motion between the current
 * LeapFrame and the specified LeapFrame is intended to be a rotating motion.
 *
 * @example Hand_rotationProbability.txt
 *
 * If a corresponding LeapHand object is not found in sinceFrame, or if either
 * this frame or sinceFrame are invalid LeapFrame objects, then this method
 * returns zero.
 *
 * @param sinceFrame The starting LeapFrame for computing the relative rotation.
 * @returns A value between 0 and 1 representing the estimated probability
 * that the hand motion between the current frame and the specified frame
 * is intended to be a rotating motion.
 * @available Since 1.0
 */
- (float)rotationProbability:(const LeapFrame *)sinceFrame;
/**
 * The scale factor derived from this hand's motion between the current
 * LeapFrame and the specified LeapFrame.
 *
 * The scale factor is always positive. A value of 1.0 indicates no
 * scaling took place. Values between 0.0 and 1.0 indicate contraction
 * and values greater than 1.0 indicate expansion.
 *
 * The Leap derives scaling from the relative inward or outward motion of
 * a hand and its associated fingers (independent of translation
 * and rotation).
 *
 * @example Hand_scale.txt
 *
 * If a corresponding LeapHand object is not found in sinceFrame, or if either
 * this frame or sinceFrame are invalid LeapFrame objects, then this method
 * returns 1.0.
 *
 * @param sinceFrame The starting LeapFrame for computing the relative scaling.
 * @returns A positive value representing the heuristically determined
 * scaling change ratio of the hand between the current frame and that
 * specified in the sinceFrame parameter.
 * @available Since 1.0
 */
- (float)scaleFactor:(const LeapFrame *)sinceFrame;
/**
 * The estimated probability that the hand motion between the current
 * LeapFrame and the specified LeapFrame is intended to be a scaling motion.
 *
 * @example Hand_scaleProbability.txt
 *
 * If a corresponding LeapHand object is not found in sinceFrame, or if either
 * this frame or sinceFrame are invalid LeapFrame objects, then this method
 * returns zero.
 *
 * @param sinceFrame The starting LeapFrame for computing the relative scaling.
 * @returns A value between 0 and 1 representing the estimated probability
 * that the hand motion between the current frame and the specified frame
 * is intended to be a scaling motion.
 * @available Since 1.0
 */
- (float)scaleProbability:(const LeapFrame *)sinceFrame;
/**
 * The duration of time this Hand has been visible to the Leap Motion Controller.
 *
 * @example Hand_timeVisible.txt
 *
 * @returns The duration (in seconds) that this Hand has been tracked.
 * @available Since 1.0
 */
@property (nonatomic, getter = timeVisible, readonly)float timeVisible;
- (float)timeVisible;
/**
 * Rates how well the internal hand model fits the observed data.
 *
 * The confidence level ranges between 0.0 and 1.0 inclusive, with 1.0 representing
 * high confidence.
 *
 * @example Hand_confidence.txt
 *
 * @returns A confidence rating between 0 and 1.
 * @available Since 2.0
 */
@property (nonatomic, getter = confidence, readonly)float confidence;
- (float)confidence;

/**
 * Identifies whether this Hand is a left hand.
 *
 * @example Hand_isLeft.txt
 *
 * @returns True if the hand is a left hand.
 * @available Since 2.0
 */
@property (nonatomic, getter = isLeft, readonly)BOOL isLeft;
- (BOOL)isLeft;
/**
 * Identifies whether this Hand is a right hand.
 *
 * @example Hand_isRight.txt
 *
 * @returns True if the hand is a right hand.
 * @available Since 2.0
 */
@property (nonatomic, getter = isRight, readonly)BOOL isRight;
- (BOOL)isRight;
/**
 * The orientation of the hand as a basis matrix.
 *
 * The basis is defined as follows:
 *
 * **xAxis** Positive in the direction of the pinky
 *
 * **yAxis** Positive above the hand
 *
 * **zAxis** Positive in the direction of the wrist
 *
 * Note: Since the left hand is a mirror of the right hand, the
 * basis matrix uses the left-hand rule for left hands.
 *
 * @example Hand_Finger_Transform.txt
 *
 * @returns The basis of the hand as a matrix.
 * @since 2.0
 */
@property (nonatomic, getter = basis, readonly)LeapMatrix *basis;
- (LeapMatrix *)basis;
/**
 * The estimated width of the palm when the hand is in a flat position.
 *
 * @example Hand_palmWidth.txt
 *
 * @returns The width of the palm in millimeters
 * @since 2.0
 */
@property (nonatomic, getter = palmWidth, readonly)float palmWidth;
- (float)palmWidth;
/**
 * Returns an invalid LeapHand object.
 *
 * @example Hand_invalid.txt
 *
 * @returns The invalid LeapHand instance.
 * @available Since 1.0
 */
+ (LeapHand *)invalid;

@end

//////////////////////////////////////////////////////////////////////////
//POINTABLEorHANDLIST CATEGORY
/**
 * The LeapPointableOrHandList category provides methods for getting objects
 * from an NSArray containing LeapPointable, LeapFinger, LeapTool, or
 * LeapHand objects based on their physical position within the Leap
 * coordinate system.
 *
 * @example List_Basic.txt
 *
 * @available Since 1.0
 */
@interface NSArray (LeapPointableOrHandList)
/**
 * The member of the list that is farthest to the left within the standard
 * Leap frame of reference (i.e has the smallest X coordinate).
 *
 * @example ListCategory_leftmost.txt
 *
 * Returns nil if the NSArray is empty.
 * @available Since 1.0
 */
- (id)leftmost;
/**
 * The member of the list that is farthest to the right within the standard
 * Leap frame of reference (i.e has the largest X coordinate).
 *
 * @example ListCategory_rightmost.txt
 *
 * Returns nil if the NSArray is empty.
 * @available Since 1.0
 */
- (id)rightmost;
/**
 * The member of the list that is farthest to the front within the standard
 * Leap frame of reference (i.e has the smallest Z coordinate).
 *
 * @example ListCategory_frontmost.txt
 *
 * Returns nil if the NSArray is empty.
 * @available Since 1.0
 */
- (id)frontmost;
/**
 * Returns a new list containing those members of the current list that are
 * extended. This includes all tools and any fingers whose [LeapPointable isExtended] function is true.
 * Unlike its C++ counterpart, this method does not modify the original NSArray.
 *
 * @example ListCategory_extended.txt
 *
 * @returns The list of tools and extended fingers from the current list.
 * @available Since 2.0
 */
- (NSArray *)extended;
/**
 * Returns a new list containing those Pointable objects in the current list
 * that are of the specified finger type.
 *
 * @example ListCategory_fingerType.txt
 *
 * @returns The list of matching fingers from the current list.
 * @available Since 2.0
 */
- (NSArray *)fingerType:(LeapFingerType)type;

@end

//////////////////////////////////////////////////////////////////////////
//SCREEN
/*
 * Deprecated as of version 1.2.
 */
@interface LeapScreen : NSObject

- (NSString *)description;
- (int32_t)id;
@property (nonatomic, getter = id, readonly)int32_t id;
- (LeapVector *)intersect:(LeapPointable *)pointable normalize:(BOOL)normalize clampRatio:(float)clampRatio;
- (LeapVector *)intersect:(const LeapVector *)position direction:(const LeapVector *)direction normalize:(BOOL)normalize clampRatio:(float)clampRatio;
- (LeapVector *)project:(LeapVector *)position normalize:(BOOL)normalize clampRatio:(float)clampRatio;
- (LeapVector *)horizontalAxis;
@property (nonatomic, getter = horizontalAxis, readonly)LeapVector *horizontalAxis;
- (LeapVector *)verticalAxis;
@property (nonatomic, getter = verticalAxis, readonly)LeapVector *verticalAxis;
- (LeapVector *)bottomLeftCorner;
@property (nonatomic, getter = bottomLeftCorner, readonly)LeapVector *bottomLeftCorner;
- (LeapVector *)normal;
@property (nonatomic, getter = normal, readonly)LeapVector *normal;
- (int)widthPixels;
@property (nonatomic, getter = widthPixels, readonly)int widthPixels;
- (int)heightPixels;
@property (nonatomic, getter = heightPixels, readonly)int heightPixels;
- (float)distanceToPoint:(const LeapVector *)point;
- (BOOL)isValid;
@property (nonatomic, getter = isValid, readonly)BOOL isValid;
- (BOOL)equals:(const LeapScreen *)other;
+ (LeapScreen *)invalid;
@end

//////////////////////////////////////////////////////////////////////////
// SCREENLIST Category
/*
 * Deprecated as of version 1.2.
 */
@interface NSArray (LeapScreenList)
- (LeapScreen *)closestScreenHit:(LeapPointable *)pointable;
- (LeapScreen *)closestScreenHit:(const LeapVector *)position direction:(const LeapVector *)direction;
- (LeapScreen *)closestScreen:(LeapVector *)position;
@end

//////////////////////////////////////////////////////////////////////////
//DEVICE
/**
 * The available types of Leap Motion controllers.
 * @since 1.2
 */
typedef enum DeviceType
{
 /**
  * A standalone USB peripheral. The original Leap Motion controller device.
  * @since 1.2
  */
    DEVICE_PERIPHERAL = 1,
 /**
  * A controller embedded in a keyboard.
  * @since 1.2
  */
    DEVICE_KEYBOARD,
 /**
  * A controller embedded in a laptop computer.
  * @since 1.2
  */
    DEVICE_LAPTOP
} LeapDeviceType;

/**
 * The LeapDevice class represents a physically connected device.
 *
 * The LeapDevice class contains information related to a particular connected
 * device such as field of view, device id, and calibrated positions.
 *
 * @example Device_get.txt
 *
 * Note that Device objects can be invalid, which means that they do not contain
 * valid device information and do not correspond to a physical device.
 * Test for validity with the [LeapDevice isValid] function.
 * @available Since 1.0
 */
@interface LeapDevice : NSObject

- (NSString *)description;
/**
 * The angle of view along the x axis of this device, in radians.
 *
 * <img src="../docs/images/Leap_horizontalViewAngle.png"/>
 *
 * The Leap Motion controller scans a volume in the shape of an inverted pyramid
 * centered at the device's center and extending upwards. The horizontalViewAngle
 * reports the view angle along the long dimension of the device.
 *
 * @example Device_horizontalViewAngle.txt
 *
 * @available Since 1.0
 */
@property (nonatomic, getter = horizontalViewAngle, readonly)float horizontalViewAngle;
- (float)horizontalViewAngle;
/**
 * The angle of view along the z axis of this device, in radians.
 *
 * <img src="../docs/images/Leap_verticalViewAngle.png"/>
 *
 * The Leap Motion controller scans a region in the shape of an inverted pyramid
 * centered at the device's center and extending upwards. The verticalViewAngle
 * reports the view angle along the short dimension of the device.
 *
 * @example Device_verticalViewAngle.txt
 *
 * @available Since 1.0
 */
@property (nonatomic, getter = verticalViewAngle, readonly)float verticalViewAngle;
- (float)verticalViewAngle;
/**
 * The maximum reliable tracking range, in millimeters.
 *
 * The range reports the maximum recommended distance from the device center
 * for which tracking is expected to be reliable. This distance is not a hard limit.
 * Tracking may be still be functional above this distance or begin to degrade slightly
 * before this distance depending on calibration and extreme environmental conditions.
 *
 * @example Device_range.txt
 *
 * @available Since 1.0
 */
@property (nonatomic, getter = range, readonly)float range;
- (float)range;
/**
  * The device baseline refers to the separation distance between stereo sensors on the
  * device. The baseline value, together with the maximum resolution, influence the
  * maximum range.
  *
  * @example Device_baseline.txt
  *
  * @available Since 2.2.5
  */
@property (nonatomic, getter = baseline, readonly)float baseline;
- (float)baseline;
/**
 * The distance to the nearest edge of the Leap Motion controller's view volume.
 *
 * The view volume is an axis-aligned, inverted pyramid centered on the device origin
 * and extending upward to the range limit. The walls of the pyramid are described
 * by the horizontalViewAngle and verticalViewAngle properties and the roof by the range.
 * This function estimates the distance between the specified input position and the
 * nearest wall or roof of the view volume.
 *
 * @example Device_distanceToBoundary.txt
 *
 * @param position The point to use for the distance calculation.
 * @returns The distance in millimeters from the input position to the nearest boundary.
 * @available Since 1.0
 */
- (float)distanceToBoundary:(const LeapVector *)position;
/**
 * Reports whether this device is embedded in another computer or computer
 * peripheral.
 *
 * @example Device_isEmbedded.txt
 *
 * @returns True, if this device is embedded in a laptop, keyboard, or other computer
 * component; false, if this device is a standalone controller.
 * @since 1.2
 */
@property(nonatomic, getter = isEmbedded, readonly)BOOL isEmbedded;
- (BOOL)isEmbedded;

/**
 * Reports whether this device is streaming data to your application.
 *
 * @example Device_isStreaming.txt
 *
 * Currently only one controller can provide data at a time.
 * @since 1.2
 */
@property(nonatomic, getter = isStreaming, readonly)BOOL isStreaming;
- (BOOL)isStreaming;

/**
 * Deprecated. Always reports false.
 *
 * @since 2.1
 * @deprecated 2.1.1
 */
@property(nonatomic, getter = isFlipped, readonly)BOOL isFlipped;
- (BOOL)isFlipped;

/**
 * The device type.
 *
 * Use the device type value in the (rare) circumstances that you
 * have an application feature which relies on a particular type of device.
 * Current types of device include the original Leap Motion peripheral,
 * keyboard-embedded controllers, and laptop-embedded controllers.
 *
 * @example Device_type.txt
 *
 * @returns The physical device type as a member of the DeviceType enumeration.
 * @since 1.2
 */
@property(nonatomic, getter = type, readonly)LeapDeviceType type;
- (LeapDeviceType)type;

/**
 * An alphanumeric serial number unique to each device.
 *
 * Consumer device serial numbers consist of 2 letters followed by 11 digits.
 *
 * When using multiple devices, the serial number provides an unambiguous
 * identifier for each device.
 * @since 2.2.2
 */
@property(nonatomic, getter = serialNumber, readonly)NSString *serialNumber;
-(NSString *) serialNumber;

/*
 * This API is experimental and not currently intended for external use.
 * Position and orientation can only be manually configured via a config file.
 * This API and the config file may change in the future or be removed entirely.
 *
 * The position of the center of the device in global coordinates (currently defined
 * in the configuration file).
 * @since 2.2.2
 */
@property(nonatomic, getter = position, readonly)LeapVector *position;
- (LeapVector *) position;

/*
 * This API is experimental and not currently intended for external use.
 * Position and orientation can only be manually configured via a config file.
 * This API and the config file may change in the future or be removed entirely.
 *
 * The orientation of the device is described by a right-handed basis:
 * xBasis : Unit vector along baseline axis between camera centers
 * yBasis : Unit vector in the direction of the center of view of both cameras
 * zBasis : The completion of the right-handed basis (perpendicular to the
 *          x and y vectors)
 *
 * In the case of a peripheral device, the z-basis vector points
 * out from the green-status-LED side of the device. When multiple-device
 * tracking is enabled, automatic coordinate system orientation is disabled.
 *
 * \image html images/Leap_Axes.png
 *
 * @since 2.2.2
*/
@property(nonatomic, getter = orientation, readonly)LeapMatrix *orientation;
- (LeapMatrix *) orientation;

/**
 * Reports whether this is a valid LeapDevice object.
 *
 * @example Device_isValid.txt
 *
 * @returns True, if this LeapDevice object contains valid data.
 * @available Since 1.0
 */
@property (nonatomic, getter = isValid, readonly)BOOL isValid;
- (BOOL)isValid;
/**
 * Compare LeapDevice object inequality.
 * Two LeapDevice objects are equal if and only if both LeapDevice objects represent the
 * exact same LeapDevice and both Devices are valid.
 *
 * @example Device_equals.txt
 *
 * @available Since 1.0
 */
- (BOOL)equals:(const LeapDevice *)other;
// not provided: not_equals
// user should emulate with !scr.equals(...)
/**
 * Returns an invalid LeapDevice object.
 *
 * @example Device_invalid.txt
 *
 * @returns The invalid LeapDevice instance.
 * @available Since 1.0
 */
+ (LeapDevice *)invalid;

@end

///////
//////////////////////////////////////////////////////////////////////////
//IMAGE
/**
* The LeapImage class represents a single greyscale image from one of the the Leap Motion cameras.
*
* In addition to image data, the LeapImage object provides a distortion map for correcting
* lens distortion.
*
* Note that LeapImage objects can be invalid, which means that they do not contain
* valid image data. Get valid LeapImage objects from [LeapFrame frames:]. Test for
* validity with the LeapImage.isValid.
* @since 2.1.5
*/
@interface LeapImage : NSObject

- (NSString *)description;
/**
 * The image ID.
 *
 * Images with ID of 0 are from the left camera; those with an ID of 1 are from the
 * right camera (with the device in its standard operating position with the
 * green LED facing the operator).
 *
 * @since 2.1.5
 */
- (int32_t)id;
/**
 * The image sequence ID.
 *
 * \include Image_sequenceId.txt
 *
 * @since 2.2.1
 */
- (int64_t)sequenceId;
/**
 * The image width.
 *
 * \include Image_image_width_1.txt
 *
 * @since 2.1.5
 */
@property (nonatomic, getter = width, readonly)int width;
- (int)width;
/**
 * The image height.
 *
 * \include Image_image_height_1.txt
 *
 * @since 2.1.5
 */
@property (nonatomic, getter = height, readonly)int height;
- (int)height;
/**
 * The number of bytes per pixel.
 *
 * \include Image_bytesPerPixel.txt
 *
 * @since 2.2.0
 */
@property (nonatomic, getter = bytesPerPixel, readonly)int bytesPerPixel;
- (int)bytesPerPixel;
/**
 * The image format.
 *
 * \include Image_format.txt
 *
 * @since 2.2.0
 */
@property (nonatomic, getter = format, readonly)LeapImageFormatType format;
- (LeapImageFormatType)format;
/**
 * The stride of the distortion map.
 *
 * Since each point on the 64x64 element distortion map has two values in the
 * buffer, the stride is 2 times the size of the grid. (Stride is currently fixed
 * at 2 * 64 = 128).
 *
 * \include Image_distortion_width_1.txt
 *
 * @since 2.1.5
 */
@property (nonatomic, getter = distortionWidth, readonly)int distortionWidth;
- (int)distortionWidth;
/**
 * The distortion map height.
 *
 * Currently fixed at 64.
 *
 * \include Image_distortion_height_1.txt
 *
 * @since 2.1.5
 */
@property (nonatomic, getter = distortionHeight, readonly)int distortionHeight;
- (int)distortionHeight;
/**
 * The horizontal ray offset.
 *
 * Used to convert between normalized coordinates in the range [0..1] and the
 * ray slope range [-4..4].
 *
 * \include Image_ray_factors_1.txt
 *
 * @since 2.1.5
 */
@property (nonatomic, getter = rayOffsetX, readonly)float rayOffsetX;
- (float)rayOffsetX;
/**
 * The vertical ray offset.
 *
 * Used to convert between normalized coordinates in the range [0..1] and the
 * ray slope range [-4..4].
 *
 * \include Image_ray_factors_2.txt
 *
 * @since 2.1.5
 */
@property (nonatomic, getter = rayOffsetY, readonly)float rayOffsetY;
- (float)rayOffsetY;
/**
 * The horizontal ray scale factor.
 *
 * Used to convert between normalized coordinates in the range [0..1] and the
 * ray slope range [-4..4].
 *
 * \include Image_ray_factors_1.txt
 *
 * @since 2.1.5
 */
@property (nonatomic, getter = rayScaleX, readonly)float rayScaleX;
- (float)rayScaleX;
/**
 * The vertical ray scale factor.
 *
 * Used to convert between normalized coordinates in the range [0..1] and the
 * ray slope range [-4..4].
 *
 * \include Image_ray_factors_2.txt
 *
 * @since 2.1.5
 */
@property (nonatomic, getter = rayScaleY, readonly)float rayScaleY;
- (float)rayScaleY;

/**
 * Returns a timestamp indicating when this frame began being captured on the device.
 * 
 * @since 2.2.7
 */
@property (nonatomic, getter = timestamp, readonly)int64_t timestamp;
- (int64_t)timestamp;

/**
 * A pointer to the image data.
 *
 * The image data is a set of 8-bit intensity values. The size of the buffer is
 * ``image.width * image.height`` bytes.
 *
 * \include Image_data_1.txt
 *
 * Where convenient, you can wrap the image data in an NSData object:
 *
 * \include Image_nsdatawrap.txt
 *
 * @since 2.1.5
 */
@property (nonatomic, getter = data, readonly) const unsigned char *data;
- (const unsigned char *) data;

/**
 * A pointer to the distortion calibration map.
 *
 * The calibration map is a 64x64 grid of points. Each point is defined by
 * a pair of 32-bit floating point values. Thus the size of the buffer is
 * ``(image.distortionWidth * 2) * image.distortionHeight * 4 bytes``.
 * Currently the size is not dynamic and evaluates to: ``64 * 2 * 64 * 4 = 32768 bytes``.
 * A future device or change in the
 * API, however, could result in a different size for the calibration map.
 * You should always use the ``image.distortionWidth`` and ``image.distortionHeight``
 * properties to calculate the buffer size.
 *
 * Each point in the map
 * represents a ray projected into the camera. The value of
 * a grid point defines the pixel in the image data containing the brightness
 * value produced by the light entering along the corresponding ray. By
 * interpolating between grid data points, you can find the brightness value
 * for any projected ray. Grid values that fall outside the range [0..1] do
 * not correspond to a value in the image data and those points should be ignored.
 *
 * \include Image_distortion_1.txt
 *
 * The calibration map can be used to render an undistorted image as well as to
 * find the true angle from the camera to a feature in the raw image. The
 * distortion map itself is designed to be used with GLSL shader programs.
 * In other contexts, it may be more convenient to use the [LeapImage rectify:]
 * and [LeapImage warp:] functions.
 *
 * Distortion is caused by the lens geometry as well as imperfections in the
 * lens and sensor window. The calibration map is created by the calibration
 * process run for each device at the factory (and which can be rerun by the
 * user).
 *
 * @since 2.1.5
 */
@property (nonatomic, getter = distortion, readonly)const float *distortion;
- (const float *) distortion;

/**
 * Provides the corrected camera ray intercepting the specified point on the image.
 *
 * Given a point on the image, ``rectify:`` corrects for camera distortion
 * and returns the true direction from the camera to the source of that image point
 * within the Leap Motion field of view.
 *
 * This direction vector has an x and y component with the third element
 * always zero: i.e. [x, y, 0]. Note that this vector uses the 2D camera coordinate system
 * where the x-axis parallels the longer (typically horizontal) dimension and
 * the y-axis parallels the shorter (vertical) dimension. The camera coordinate
 * system does not correlate to the Leap Motion 3D coordinate system.
 *
 * \include Image_rectify_1.txt
 *
 * @param uv A LeapVector containing the position of a pixel in the image.
 * @returns A LeapVector containing the ray direction (the z-component of the vector is always 0).
 * @since 2.1.5
 */
- (LeapVector *)rectify:(LeapVector *)uv;
/**
 * Provides the point in the image corresponding to a ray projecting
 * from the camera.
 *
 * Given a ray projected from the camera in the specified direction, ``warp:``
 * corrects for camera distortion and returns the corresponding pixel
 * coordinates in the image.
 *
 * The ray direction is specified in relationship to the camera. The first
 * vector element corresponds to the "horizontal" view angle; the second
 * corresponds to the "vertical" view angle.
 *
 * \include Image_warp_1.txt
 *
 * @param xy A LeapVector containing the ray direction.
 * @returns A LeapVector containing the pixel coordinates [x, y, 0] (with z always zero).
 * @since 2.1.5
 */
- (LeapVector *)warp:(LeapVector *)xy;
/**
 * Reports whether this LeapImage instance contains valid data.
 *
 * @returns true, if and only if the image is valid.
 * @since 2.1.5
 */
@property (nonatomic, getter = isValid, readonly)BOOL isValid;
- (BOOL)isValid;
/**
 * Compare LeapImage object equality.
 *
 * Two LeapImage objects are equal if and only if both objects represent the
 * exact same image and both LeapImage objects are valid.
 * @since 2.1.5
 */
- (BOOL)equals:(const LeapImage *)other;
/**
 * Returns an invalid Image object.
 *
 * You can use the instance returned by this function in comparisons testing
 * whether a given Image instance is valid or invalid. (You can also use the
 * LeapImage.isValid property.)
 *
 *
 * @returns The invalid LeapImage instance.
 * @since 2.1.5
 */
+ (LeapImage *)invalid;
@end
///////

//////////////////////////////////////////////////////////////////////////
//INTERACTIONBOX
/**
 * The LeapInteractionBox class represents a box-shaped region completely
 * within the field of view of the Leap Motion controller.
 *
 * The interaction box is an axis-aligned rectangular prism and provides normalized
 * coordinates for hands, fingers, and tools within this box. The InteractionBox class
 * can make it easier to map positions in the Leap Motion coordinate system to 2D or
 * 3D coordinate systems used for application drawing.
 *
 * <img src="../docs/images/Leap_InteractionBox.png"/>
 *
 * The LeapInteractionBox region is defined by a center and dimensions along the x, y,
 * and z axes.
 *
 * @example IBox_get.txt
 *
 * @available Since 1.0
 */
@interface LeapInteractionBox : NSObject

- (NSString *)description;
/**
 * Normalizes the coordinates of a point using the interaction box.
 *
 * Coordinates from the Leap frame of reference (millimeters) are converted
 * to a range of [0..1] such that the minimum value of the LeapInteractionBox maps to 0
 * and the maximum value of the LeapInteractionBox maps to 1.
 *
 * @example IBox_normalizePoint.txt
 *
 * @param position The input position in device coordinates.
 * @param clamp Whether or not to limit the output value to the range [0,1] when the
 * input position is outside the LeapInteractionBox. Defaults to true.
 * @returns The normalized position.
 * @available Since 1.0
 */
- (LeapVector *)normalizePoint:(const LeapVector *)position clamp:(BOOL)clamp;
/**
 * Converts a position defined by normalized LeapInteractionBox coordinates into device
 * coordinates in millimeters.
 *
 * @example IBox_denormalizePoint.txt
 *
 * This function performs the inverse of [LeapInteractionBox normalizePoint:].
 *
 * @param position A normalized position in LeapInteractionBox coordinates.
 * @returns The corresponding denormalized position in device coordinates.
 * @available Since 1.0
 */
- (LeapVector *)denormalizePoint:(const LeapVector *)position;
/**
 * The center of the LeapInteractionBox in device coordinates (millimeters). This point
 * is equidistant from all sides of the box.
 *
 * @example IBox_center.txt
 *
 * @returns The LeapInteractionBox center in device coordinates.
 * @available Since 1.0
 */
@property (nonatomic, getter = center, readonly)LeapVector *center;
- (LeapVector *)center;
/**
 * The width of the LeapInteractionBox in millimeters, measured along the x-axis.
 *
 * @example IBox_width.txt
 *
 * @returns The LeapInteractionBox width in millimeters.
 * @available Since 1.0
 */
@property (nonatomic, getter = width, readonly)float width;
- (float)width;
/**
 * The height of the LeapInteractionBox in millimeters, measured along the y-axis.
 *
 * @example IBox_height.txt
 *
 * @returns The LeapInteractionBox height in millimeters.
 * @available Since 1.0
 */
@property (nonatomic, getter = width, readonly)float height;
- (float)height;
/**
 * The depth of the LeapInteractionBox in millimeters, measured along the z-axis.
 *
 * @example IBox_depth.txt
 *
 * @returns The LeapInteractionBox depth in millimeters.
 * @available Since 1.0
 */
@property (nonatomic, getter = width, readonly)float depth;
- (float)depth;
/**
 * Reports whether this is a valid LeapInteractionBox object.
 *
 * @example IBox_isValid.txt
 *
 * @returns True, if this LeapInteractionBox object contains valid data.
 * @available Since 1.0
 */
@property (nonatomic, getter = isValid, readonly)BOOL isValid;
- (BOOL)isValid;
/**
 * Reports whether this is a valid LeapInteractionBox object.
 *
 * @example IBox_equals.txt
 *
 * @returns True, if this LeapInteractionBox object contains valid data.
 * @available Since 1.0
 */
- (BOOL)equals:(const LeapInteractionBox *)other;
// not provided: not_equals
// user should emulate with !scr.equals(...)
/**
 * Returns an invalid LeapInteractionBox object.
 *
 * @example IBox_invalid.txt
 *
 * @returns The invalid InteractionBox instance.
 * @available Since 1.0
 */
+ (LeapInteractionBox *)invalid;

@end

//////////////////////////////////////////////////////////////////////////
//GESTURE
/**
 * The LeapGesture class represents a recognized movement by the user.
 *
 * The Leap watches the activity within its field of view for certain movement
 * patterns typical of a user gesture or command. For example, a movement from side to
 * side with the hand can indicate a swipe gesture, while a finger poking forward
 * can indicate a screen tap gesture.
 *
 * When the Leap recognizes a gesture, it assigns an ID and adds a
 * LeapGesture object to the frame gesture list. For continuous gestures, which
 * occur over many frames, the Leap updates the gesture by adding
 * a LeapGesture object having the same ID and updated properties in each
 * subsequent frame.
 *
 * **Important:** Recognition for each type of gesture must be enabled using the
 * [LeapController enableGesture:enable:] function; otherwise **no gestures are
 * recognized or reported**.
 *
 * @example Gesture_Enable_All.txt
 *
 * Subclasses of LeapGesture define the properties for the specific movement patterns
 * recognized by the Leap.
 *
 * The LeapGesture subclasses for include:
 *
 * **LeapCircleGesture** -- A circular movement by a finger.
 *
 * **LeapSwipeGesture** -- A straight line movement by the hand with fingers extended.
 *
 * **LeapScreenTapGesture** -- A forward tapping movement by a finger.
 *
 * **LeapKeyTapGesture** -- A downward tapping movement by a finger.
 *
 * Circle and swipe gestures are continuous and these objects can have a
 * state of start, update, and stop.
 *
 * The tap gestures are discrete. The Leap only creates a single
 * LeapScreenTapGesture or LeapKeyTapGesture object appears for each tap and that
 * object is always assigned the stop state.
 *
 * Get valid LeapGesture instances from a LeapFrame object. You can get a list of gestures
 * with the [LeapFrame gestures:] method. You can also
 * use the [LeapFrame gesture:] method to find a gesture in the current frame using
 * an ID value obtained in a previous frame.
 *
 * LeapGesture objects can be invalid. For example, when you get a gesture by ID
 * using `[LeapFrame gesture:]`, and there is no gesture with that ID in the current
 * frame, then `gesture:` returns an Invalid LeapGesture object (rather than a null
 * value). Always check object validity in situations where a gesture might be
 * invalid.
 *
 * The following keys can be used with the LeapConfig class to configure the gesture
 * recognizer:
 *
 * \table
 * ====================================  ========== ============= =======
 * Key string                            Value type Default value Units
 * ====================================  ========== ============= =======
 * Gesture.Circle.MinRadius              float      5.0           mm
 * Gesture.Circle.MinArc                 float      1.5 * pi      radians
 * Gesture.Swipe.MinLength               float      150           mm
 * Gesture.Swipe.MinVelocity             float      1000          mm/s
 * Gesture.KeyTap.MinDownVelocity        float      50            mm/s
 * Gesture.KeyTap.HistorySeconds         float      0.1           s
 * Gesture.KeyTap.MinDistance            float      3.0           mm
 * Gesture.ScreenTap.MinForwardVelocity  float      50            mm/s
 * Gesture.ScreenTap.HistorySeconds      float      0.1           s
 * Gesture.ScreenTap.MinDistance         float      5.0           mm
 * ====================================  ========== ============= =======
 * \endtable
 *
 * The LeapController object must be connected to the Leap Motion service/daemon
 * before setting the configuration parameters.
 *
 * @example Gesture_Params_All.txt
 *
 * @available Since 1.0
 */
@interface LeapGesture : NSObject
/**
 * The LeapFrame containing this LeapGesture instance.
 *
 * @example Gesture_frame.txt
 *
 * @return The parent LeapFrame object.
 * @available Since 1.0
 */
@property (nonatomic, strong, readonly)LeapFrame *frame;
/**
 * The list of hands associated with this LeapGesture, if any.
 *
 * @example Gesture_hands.txt
 *
 * If no hands are related to this gesture, the list is empty.
 *
 * @return NSArray the list of related LeapHand objects.
 * @available Since 1.0
 */
@property (nonatomic, strong, readonly)NSArray *hands;
/**
 * The list of fingers and tools associated with this LeapGesture, if any.
 *
 * @example Gesture_pointables.txt
 *
 * If no LeapPointable objects are related to this gesture, the list is empty.
 *
 * @return NSArray the list of related LeapPointable objects.
 * @available Since 1.0
 */
@property (nonatomic, strong, readonly)NSArray *pointables;

- (NSString *)description;
/**
 * The gesture type.
 *
 * @example Gesture_type.txt
 *
 * The supported types of gestures are defined by the LeapGestureType enumeration:
 *
 * **LEAP_GESTURE_TYPE_INVALID** -- An invalid type.
 *
 * **LEAP_GESTURE_TYPE_SWIPE**  -- A straight line movement by the hand with fingers extended.
 *
 * **LEAP_GESTURE_TYPE_CIRCLE** -- A circular movement by a finger.
 *
 * **LEAP_GESTURE_TYPE_SCREEN_TAP** -- A forward tapping movement by a finger.
 *
 * **LEAP_GESTURE_TYPE_KEY_TAP** -- A downward tapping movement by a finger.
 *
 * @returns LeapGestureType A value from the LeapGestureType enumeration.
 * @available Since 1.0
 */
@property (nonatomic, getter = type, readonly)LeapGestureType type;
- (LeapGestureType)type;
/**
 * The gesture state.
 *
 * Recognized movements occur over time and have a beginning, a middle,
 * and an end. The 'state' attribute reports where in that sequence this
 * LeapGesture object falls.
 *
 * @example Gesture_state.txt
 *
 * The possible gesture states are defined by the LeapGestureState enumeration:
 *
 * **LEAP_GESTURE_STATE_INVALID** -- An invalid state.
 *
 * **LEAP_GESTURE_STATE_START** -- The gesture is starting. Just enough has happened to recognize it.
 *
 * **LEAP_GESTURE_STATE_UPDATE** -- The gesture is in progress. (Note: not all gestures have updates).
 *
 * **LEAP_GESTURE_STATE_STOP** -- The gesture has completed or stopped.
 *
 * @returns LeapGestureState A value from the LeapGestureState enumeration.
 * @available Since 1.0
 */
@property (nonatomic, getter = state, readonly)LeapGestureState state;
- (LeapGestureState)state;
/**
 * The gesture ID.
 *
 * All LeapGesture objects belonging to the same recognized movement share the
 * same ID value. Use the ID value with the [LeapFrame gesture:] method to
 * find updates related to this LeapGesture object in subsequent frames.
 *
 * @example Gesture_id.txt
 *
 * @returns int32_t the ID of this LeapGesture.
 * @available Since 1.0
 */
@property (nonatomic, getter = id, readonly)int32_t id;
- (int32_t)id;
/**
 * The elapsed duration of the recognized movement up to the
 * frame containing this LeapGesture object, in microseconds.
 *
 * @example Gesture_duration.txt
 *
 * The duration reported for the first LeapGesture in the sequence (with the
 * LEAP_GESTURE_STATE_START state) will typically be a small positive number since
 * the movement must progress far enough for the Leap to recognize it as
 * an intentional gesture.
 *
 * @return int64_t the elapsed duration in microseconds.
 * @available Since 1.0
 */
@property (nonatomic, getter = duration, readonly)int64_t duration;
- (int64_t)duration;
/**
 * The elapsed duration in seconds.
 *
 * @example Gesture_durationSeconds.txt
 *
 * @see duration
 * @return float the elapsed duration in seconds.
 * @available Since 1.0
 */
@property (nonatomic, getter = durationSeconds, readonly)float durationSeconds;
- (float)durationSeconds;
/**
 * Reports whether this LeapGesture instance represents a valid gesture.
 *
 * An invalid LeapGesture object does not represent a snapshot of a recognized
 * movement. Invalid LeapGesture objects are returned when a valid object cannot
 * be provided. For example, when you get an gesture by ID
 * using [LeapFrame gesture:], and there is no gesture with that ID in the current
 * frame, then `gesture:` returns an Invalid LeapGesture object (rather than a null
 * value). Always check object validity in situations where an gesture might be
 * invalid.
 *
 * @example Gesture_isValid.txt
 *
 * @returns bool Yes, if this is a valid LeapGesture instance; NO, otherwise.
 * @available Since 1.0
 */
@property (nonatomic, getter = isValid, readonly)BOOL isValid;
- (BOOL)isValid;
/**
 * Compare LeapGesture object equality.
 *
 * Two LeapGestures are equal if they represent the same snapshot of the same
 * recognized movement.
 *
 * @example Gesture_equals.txt
 *
 * @param other The LeapGesture object to compare.
 * @available Since 1.0
 */
- (BOOL)equals:(const LeapGesture *)other;
// not provided: not_equals
// user should emulate with !scr.equals(...)
/**
 * Returns an invalid LeapGesture object.
 *
 * @example Gesture_invalid.txt
 *
 * @returns The invalid LeapGesture instance.
 * @available Since 1.0
 */
+ (LeapGesture *)invalid;

@end

////////////////////////////////////////////////////////////////////////
//SWIPE GESTURE
/**
 * The LeapSwipeGesture class represents a swiping motion of fingers and tools.
 *
 * <img src="../docs/images/Leap_Gesture_Swipe.png"/>
 *
 * SwipeGesture objects are generated for each swiping finger or tool.
 * Swipe gestures are continuous; a gesture object with the same
 * ID value will appear in each frame while the gesture continues. The LeapSwipeGesture
 * objects for the gesture have three possible states:
 *
 * **LEAP_GESTURE_STATE_START** -- The gesture has just started.
 *
 * **LEAP_GESTURE_STATE_UPDATE** -- The swipe gesture is continuing.
 *
 * **LEAP_GESTURE_STATE_STOP** -- The swipe gesture is finished.
 *
 * **Important:** To use swipe gestures in your application, you must enable
 * recognition of the swipe gesture. You can enable recognition with:
 *
 * @example Gesture_Swipe_enable.txt
 *
 * You can set the minimum length and velocity required for a movement
 * to be recognized as a swipe using the config attribute of a connected
 * LeapController object. Use the following keys to configure swipe recognition:
 *
 * \table
 * ====================================  ========== ============= =======
 * Key string                            Value type Default value Units
 * ====================================  ========== ============= =======
 * Gesture.Swipe.MinLength               float      150           mm
 * Gesture.Swipe.MinVelocity             float      1000          mm/s
 * ====================================  ========== ============= =======
 * \endtable
 *
 * The following example demonstrates how to set the swipe configuration
 * parameters:
 *
 * @example Gesture_Swipe_Params.txt
 *
 * The LeapController object must be connected to the Leap Motion service/daemon
 * before setting the configuration parameters.
 *
 * @available Since 1.0
 */
@interface LeapSwipeGesture : LeapGesture

/**
 * The current position of the swipe.
 *
 * @example SwipeGesture_position.txt
 *
 * @returns LeapVector The current swipe position within the Leap frame of
 * reference, in mm.
 * @available Since 1.0
 */
@property (nonatomic, getter = position, readonly)LeapVector *position;
- (LeapVector *)position;
/**
 * The position where the swipe began.
 *
 * @example SwipeGesture_startPosition.txt
 *
 * @returns LeapVector The starting position within the Leap frame of
 * reference, in mm.
 * @available Since 1.0
 */
@property (nonatomic, getter = startPosition, readonly)LeapVector *startPosition;
- (LeapVector *)startPosition;
/**
 * The unit direction vector parallel to the swipe motion.
 *
 * @example SwipeGesture_direction.txt
 *
 * You can compare the components of the vector to classify the swipe as
 * appropriate for your application. For example, if you are using swipes
 * for two dimensional scrolling, you can compare the x and y values to
 * determine if the swipe is primarily horizontal or vertical.
 *
 * @returns LeapVector The unit direction vector representing the swipe
 * motion.
 * @available Since 1.0
 */
@property (nonatomic, getter = direction, readonly)LeapVector *direction;
- (LeapVector *)direction;
/**
 * The swipe speed in mm/second.
 *
 * @example SwipeGesture_speed.txt
 *
 * @returns float The speed of the finger performing the swipe gesture in
 * millimeters per second.
 * @available Since 1.0
 */
@property (nonatomic, getter = speed, readonly)float speed;
- (float)speed;
/**
 * The finger or tool performing the swipe gesture.
 *
 * @example SwipeGesture_pointable.txt
 *
 * @returns A LeapPointable object representing the swiping finger
 * or tool.
 * @available Since 1.0
 */
- (LeapPointable *)pointable;

@end

//////////////////////////////////////////////////////////////////////////
//CIRCLE GESTURE
/**
 * The LeapCircleGesture classes represents a circular finger movement.
 *
 * A circle movement is recognized when the tip of a finger draws a circle
 * within the Leap field of view.
 *
 * <img src="../docs/images/Leap_Gesture_Circle.png"/>
 *
 * **Important:** To use circle gestures in your application, you must enable
 * recognition of the circle gesture. You can enable recognition with:
 *
 * @example Gesture_Circle_Enable.txt
 *
 * Circle gestures are continuous. The LeapCircleGesture objects for the gesture have
 * three possible states:
 *
 * **LEAP_GESTURE_STATE_START** -- The circle gesture has just started. The movement has
 *   progressed far enough for the recognizer to classify it as a circle.
 *
 * **LEAP_GESTURE_STATE_UPDATE** -- The circle gesture is continuing.
 *
 * **LEAP_GESTURE_STATE_STOP** -- The circle gesture is finished.
 *
 * You can set the minimum radius and minimum arc length required for a movement
 * to be recognized as a circle using the config attribute of a connected
 * LeapController object. Use the following keys to configure circle recognition:
 *
 * \table
 * ====================================  ========== ============= =======
 * Key string                            Value type Default value Units
 * ====================================  ========== ============= =======
 * Gesture.Circle.MinRadius              float      5.0           mm
 * Gesture.Circle.MinArc                 float      1.5 * pi      radians
 * ====================================  ========== ============= =======
 * \endtable
 *
 * The following example demonstrates how to set the circle configuration
 * parameters:
 *
 * @example Gesture_Circle_Params.txt
 *
 * The LeapController object must be connected to the Leap Motion service/daemon
 * before setting the configuration parameters.
 *
 * @available Since 1.0
 */
@interface LeapCircleGesture : LeapGesture

/**
 * The number of times the finger tip has traversed the circle.
 *
 * Progress is reported as a positive number of the number. For example,
 * a progress value of .5 indicates that the finger has gone halfway
 * around, while a value of 3 indicates that the finger has gone around
 * the the circle three times.
 *
 * @example CircleGesture_progress.txt
 *
 * Progress starts where the circle gesture began. Since the circle
 * must be partially formed before the Leap Motion software can recognize it, progress
 * will be greater than zero when a circle gesture first appears in the
 * frame.
 *
 * @returns float A positive number indicating the gesture progress.
 * @since 1.0
 */
@property (nonatomic, getter = progress, readonly)float progress;
- (float)progress;
/**
 * The center point of the circle within the Leap frame of reference.
 *
 * @example CircleGesture_center.txt
 *
 * @returns LeapVector The center of the circle in mm from the Leap origin.
 * @available Since 1.0
 */
@property (nonatomic, getter = center, readonly)LeapVector *center;
- (LeapVector *)center;

/**
 * Returns the normal vector for the circle being traced.
 *
 * If you draw the circle clockwise, the normal vector points in the same
 * general direction as the pointable object drawing the circle. If you draw
 * the circle counterclockwise, the normal points back toward the
 * pointable. If the angle between the normal and the pointable object
 * drawing the circle is less than 90 degrees, then the circle is clockwise.
 *
 * @example CircleGesture_normal.txt
 *
 * @return LeapVector the normal vector for the circle being traced
 * @available Since 1.0
 */

@property (nonatomic, getter = normal, readonly)LeapVector *normal;
- (LeapVector *)normal;
/**
 * The radius of the circle.
 *
 * @example CircleGesture_radius.txt
 *
 * @returns The circle radius in mm.
 * @available Since 1.0
 */
@property (nonatomic, getter = radius, readonly)float radius;
- (float)radius;
/**
 * The finger performing the circle gesture.
 *
 * @example CircleGesture_pointable.txt
 *
 * @returns A LeapPointable object representing the circling finger.
 * @available Since 1.0
 */
@property (nonatomic, getter = pointable, readonly)LeapPointable *pointable;
- (LeapPointable *)pointable;

@end

//////////////////////////////////////////////////////////////////////////
//SCREEN TAP GESTURE
/**
 * The LeapScreenTapGesture class represents a tapping gesture by a finger or tool.
 *
 * A screen tap gesture is recognized when the tip of a finger pokes forward
 * and then springs back to approximately the original postion, as if
 * tapping a vertical screen. The tapping finger must pause briefly before beginning the tap.
 *
 * <img src="../docs/images/Leap_Gesture_Tap2.png"/>
 *
 * **Important:** To use screen tap gestures in your application, you must enable
 * recognition of the screen tap gesture. You can enable recognition with:
 *
 * @example Gesture_ScreenTap_Enable.txt
 *
 * LeapScreenTap gestures are discrete. The LeapScreenTapGesture object
 * representing a tap always has the state, LEAP_GESTURE_STATE_STOP. Only one
 * LeapScreenTapGesture object is created for each screen tap gesture recognized.
 *
 * You can set the minimum finger movement and velocity required for a movement
 * to be recognized as a screen tap as well as adjust the detection window for
 * evaluating the movement using the config attribute of a connected
 * LeapController object. Use the following keys to configure screen tap recognition:
 *
 * \table
 * ====================================  ========== ============= =======
 * Key string                            Value type Default value Units
 * ====================================  ========== ============= =======
 * Gesture.ScreenTap.MinForwardVelocity  float      50            mm/s
 * Gesture.ScreenTap.HistorySeconds      float      0.1           s
 * Gesture.ScreenTap.MinDistance         float      5.0           mm
 * ====================================  ========== ============= =======
 * \endtable
 *
 * The following example demonstrates how to set the screen tap configuration
 * parameters:
 *
 * @example Gesture_ScreenTap_Params.txt
 *
 * The LeapController object must be connected to the Leap Motion service/daemon
 * before setting the configuration parameters.
 *
 * @available Since 1.0
 */
@interface LeapScreenTapGesture : LeapGesture

/**
 * The position where the screen tap is registered.
 *
 * @example ScreenTapGesture_position.txt
 *
 * @return A LeapVector containing the coordinates of screen tap location.
 * @available Since 1.0
 */
@property (nonatomic, getter = position, readonly)LeapVector *position;
- (LeapVector *)position;
/**
 * The direction of finger tip motion.
 *
 * @example ScreenTapGesture_direction.txt
 *
 * @returns LeapVector A unit direction vector.
 * @available Since 1.0
 */
@property (nonatomic, getter = direction, readonly)LeapVector *direction;
- (LeapVector *)direction;
/**
 * The progess value is always 1.0 for a screen tap gesture.
 *
 * @returns float The value 1.0.
 * @available Since 1.0
 */
@property (nonatomic, getter = progress, readonly)float progress;
- (float)progress;
/**
 * The finger performing the screen tap gesture.
 *
 * @example ScreenTapGesture_pointable.txt
 *
 * @returns A LeapPointable object representing the tapping finger.
 * @available Since 1.0
 */
@property (nonatomic, getter = pointable, readonly)LeapPointable *pointable;
- (LeapPointable *)pointable;

@end

//////////////////////////////////////////////////////////////////////////
//KEY TAP GESTURE
/**
 * The LeapKeyTapGesture class represents a tapping gesture by a finger or tool.
 *
 * A key tap gesture is recognized when the tip of a finger rotates down toward the
 * palm and then springs back to approximately the original postion, as if
 * tapping. The tapping finger must pause briefly before beginning the tap.
 *
 * <img src="../docs/images/Leap_Gesture_Tap.png"/>
 *
 * **Important:** To use key tap gestures in your application, you must enable
 * recognition of the key tap gesture. You can enable recognition with:
 *
 * @example Gesture_KeyTap_Enable.txt
 *
 * Key tap gestures are discrete. The LeapKeyTapGesture object representing a tap always
 * has the state, LEAP_GESTURE_STATE_STOP. Only one LeapKeyTapGesture object is
 * created for each key tap gesture recognized.
 *
 * You can set the minimum finger movement and velocity required for a movement
 * to be recognized as a key tap as well as adjust the detection window for
 * evaluating the movement using the config attribute of a connected
 * LeapController object. Use the following configuration keys to configure key tap
 * recognition:
 *
 * \table
 * ====================================  ========== ============= =======
 * Key string                            Value type Default value Units
 * ====================================  ========== ============= =======
 * Gesture.KeyTap.MinDownVelocity        float      50            mm/s
 * Gesture.KeyTap.HistorySeconds         float      0.1           s
 * Gesture.KeyTap.MinDistance            float      3.0           mm
 * ====================================  ========== ============= =======
 * \endtable
 *
 * The following example demonstrates how to set the key tap configuration
 * parameters:
 *
 * @example Gesture_KeyTap_Params.txt
 *
 * The LeapController object must be connected to the Leap Motion service/daemon
 * before setting the configuration parameters.
 *
 * @available Since 1.0
 */
@interface LeapKeyTapGesture : LeapGesture

/**
 * The position where the key tap is registered.
 *
 * @example KeyTapGesture_position.txt
 *
 * @return A LeapVector containing the coordinates of key tap location.
 * @available Since 1.0
 */
@property (nonatomic, getter = position, readonly)LeapVector *position;
- (LeapVector *)position;
/**
 * The direction of finger tip motion.
 *
 * @example KeyTapGesture_direction.txt
 *
 * @returns LeapVector A unit direction vector.
 * @available Since 1.0
 */
@property (nonatomic, getter = direction, readonly)LeapVector *direction;
- (LeapVector *)direction;
/**
 * The progess value is always 1.0 for a key tap gesture.
 *
 * @returns float The value 1.0.
 * @available Since 1.0
 */
@property (nonatomic, getter = progress, readonly)float progress;
- (float)progress;
/**
 * The finger performing the key tap gesture.
 *
 * @example KeyTapGesture_pointable.txt
 *
 * @returns A LeapPointable object representing the tapping finger.
 * @available Since 1.0
 */
@property (nonatomic, getter = pointable, readonly)LeapPointable *pointable;
- (LeapPointable *)pointable;

@end

//////////////////////////////////////////////////////////////////////////
//FRAME
/**
 * The LeapFrame class represents a set of hand and finger tracking data detected
 * in a single frame.
 *
 * The Leap detects hands, fingers and tools within the tracking area, reporting
 * their positions, orientations and motions in frames at the Leap frame rate.
 *
 * Access LeapFrame objects through an instance of a LeapController.
 *
 * @example Frame_get.txt
 *
 * Implement a LeapListener subclass to receive a callback event when a new
 * LeapFrame is available.
 *
 * A LeapFrame object maintains strong references to its child objects (hands,
 * fingers, tools, etc). However, the child objects only maintain weak references
 * to their parent objects. Thus, you can access the full hierarchy of objects while
 * the LeapFrame object exists, but you can only access child objects once the
 * LeapFrame object has been deleted.
 *
 * @available Since 1.0
 */
@interface LeapFrame : NSObject

/**
 * The list of LeapHand objects detected in this frame, given in arbitrary order.
 * The list can be empty if no hands are detected.
 *
 * @example Frame_hands.txt
 *
 * @returns NSArray containing all LeapHand objects detected in this frame.
 * @available Since 1.0
 */
@property (nonatomic, strong, readonly)NSArray *hands;
/**
 * The list of LeapPointable objects (fingers and tools) detected in this frame,
 * given in arbitrary order. The list can be empty if no fingers or tools are detected.
 *
 * @example Frame_pointables.txt
 *
 * @returns NSArray containing all LeapPointable objects detected in this frame.
 * @available Since 1.0
 */
@property (nonatomic, strong, readonly)NSArray *pointables;
/**
 * The list of LeapFinger objects detected in this frame, given in arbitrary order.
 * The list can be empty if no fingers are detected.
 *
 * @example Frame_fingers.txt
 *
 * @returns NSArray containing all LeapFinger objects detected in this frame.
 * @available Since 1.0
 */
@property (nonatomic, strong, readonly)NSArray *fingers;
/**
 * The list of LeapTool objects detected in this frame, given in arbitrary order.
 * The list can be empty if no tools are detected.
 *
 * @example Frame_tools.txt
 *
 * @returns NSArray containing all LeapTool objects detected in this frame.
 * @available Since 1.0
 */
@property (nonatomic, strong, readonly)NSArray *tools;

/**
 *  The list of images from the Leap Motion cameras.
 *
 *  @return An NSArray object containing the camera images analyzed to create this Frame.
 *  @available Since 2.1.5
*/
@property (nonatomic, strong, readonly)NSArray *images;

- (NSString *)description;
- (void *)interfaceFrame;
/**
 * A unique ID for this LeapFrame. Consecutive frames processed by the Leap
 * have consecutive increasing values.
 *
 * @example Frame_id.txt
 *
 * @returns The frame ID.
 * @available Since 1.0
 */
@property (nonatomic, getter = id, readonly)int64_t id;
- (int64_t)id;
/**
 * The frame capture time in microseconds elapsed since an arbitrary point in 
 * time in the past.
 *
 * Use [LeapController now] to calculate the age of the frame.
 *
 * @example Frame_timestamp.txt
 *
 * @returns The timestamp in microseconds.
 * @available Since 1.0
 */
@property (nonatomic, getter = timestamp, readonly)int64_t timestamp;
- (int64_t)timestamp;
/**
 * The LeapHand object with the specified ID in this frame.
 *
 * Use the [LeapFrame hand:] function to retrieve the LeapHand object from
 * this frame using an ID value obtained from a previous frame.
 * This function always returns a LeapHand object, but if no hand
 * with the specified ID is present, an invalid LeapHand object is returned.
 *
 * @example Frame_hand.txt
 *
 * Note that ID values persist across frames, but only until tracking of a
 * particular object is lost. If tracking of a hand is lost and subsequently
 * regained, the new LeapHand object representing that physical hand may have
 * a different ID than that representing the physical hand in an earlier frame.
 *
 * @param handId The ID value of a LeapHand object from a previous frame.
 * @returns The LeapHand object with the matching ID if one exists in this frame;
 * otherwise, an invalid LeapHand object is returned.
 * @available Since 1.0
 */
- (LeapHand *)hand:(int32_t)handId;
/**
 * The LeapPointable object with the specified ID in this frame.
 *
 * Use the [LeapFrame pointable:] function to retrieve the LeapPointable object from
 * this frame using an ID value obtained from a previous frame.
 * This function always returns a LeapPointable object, but if no finger or tool
 * with the specified ID is present, an invalid LeapPointable object is returned.
 *
 * @example Frame_pointable.txt
 *
 * Note that ID values persist across frames, but only until tracking of a
 * particular object is lost. If tracking of a finger or tool is lost and subsequently
 * regained, the new LeapPointable object representing that finger or tool may have
 * a different ID than that representing the finger or tool in an earlier frame.
 *
 * @param pointableId The ID value of a LeapPointable object from a previous frame.
 * @returns The LeapPointable object with the matching ID if one exists in this frame;
 * otherwise, an invalid LeapPointable object is returned.
 * @available Since 1.0
 */
- (LeapPointable *)pointable:(int32_t)pointableId;
/**
 * The LeapFinger object with the specified ID in this frame.
 *
 * Use the [LeapFrame finger:] function to retrieve the LeapFinger object from
 * this frame using an ID value obtained from a previous frame.
 * This function always returns a LeapFinger object, but if no finger
 * with the specified ID is present, an invalid LeapFinger object is returned.
 *
 * @example Frame_finger.txt
 *
 * Note that ID values persist across frames, but only until tracking of a
 * particular object is lost. If tracking of a finger is lost and subsequently
 * regained, the new LeapFinger object representing that physical finger may have
 * a different ID than that representing the finger in an earlier frame.
 *
 * @param fingerId The ID value of a LeapFinger object from a previous frame.
 * @returns The LeapFinger object with the matching ID if one exists in this frame;
 * otherwise, an invalid LeapFinger object is returned.
 * @available Since 1.0
 */
- (LeapFinger *)finger:(int32_t)fingerId;
/**
 * The LeapTool object with the specified ID in this frame.
 *
 * Use the [LeapFrame tool:] function to retrieve the LeapTool object from
 * this frame using an ID value obtained from a previous frame.
 * This function always returns a LeapTool object, but if no tool
 * with the specified ID is present, an invalid LeapTool object is returned.
 *
 * @example Frame_tool.txt
 *
 * Note that ID values persist across frames, but only until tracking of a
 * particular object is lost. If tracking of a tool is lost and subsequently
 * regained, the new LeapTool object representing that tool may have a
 * different ID than that representing the tool in an earlier frame.
 *
 * @param toolId The ID value of a LeapTool object from a previous frame.
 * @returns The LeapTool object with the matching ID if one exists in this frame;
 * otherwise, an invalid LeapTool object is returned.
 * @available Since 1.0
 */
- (LeapTool *)tool:(int32_t)toolId;
/**
 * The gestures recognized or continuing since the specified frame.
 *
 * Circle and swipe gestures are updated every frame. Tap gestures
 * only appear in the list for a single frame.
 *
 * @example Frame_gestures.txt
 *
 * @param sinceFrame An earlier LeapFrame. Set to nil to get the gestures for
 * the current LeapFrame only.
 * @return NSArray containing the list of gestures.
 * @available Since 1.0
 */
- (NSArray *)gestures:(const LeapFrame *)sinceFrame;
/**
 * The LeapGesture object with the specified ID in this frame.
 *
 * Use the [LeapFrame gesture:] function to return a Gesture object in this
 * frame using an ID obtained in an earlier frame. The function always
 * returns a LeapGesture object, but if there was no update for the gesture in
 * this frame, then an invalid LeapGesture object is returned.
 *
 * All LeapGesture objects representing the same recognized movement share the
 * same ID.
 *
 * @example Gesture_isValid.txt
 *
 * @param gestureId The ID of a LeapGesture object from a previous frame.
 * @returns The LeapGesture object in the frame with the specified ID if one
 * exists; Otherwise, an Invalid LeapGesture object.
 * @available Since 1.0
 */
- (LeapGesture *)gesture:(int32_t)gestureId;
/**
 * The change of position derived from the overall linear motion between
 * the current frame and the specified frame.
 *
 * The returned translation vector provides the magnitude and direction of
 * the movement in millimeters.
 *
 * The Leap derives frame translation from the linear motion of
 * all objects detected in the field of view.
 *
 * @example Frame_translation.txt
 *
 * If either this frame or sinceFrame is an invalid LeapFrame object, then this
 * method returns a zero vector.
 *
 * @param sinceFrame The starting frame for computing the relative translation.
 * @returns A LeapVector representing the heuristically determined change in
 * position of all objects between the current frame and that specified
 * in the sinceFrame parameter.
 * @available Since 1.0
 */
- (LeapVector *)translation:(const LeapFrame *)sinceFrame;
/**
 * The estimated probability that the overall motion between the current
 * frame and the specified frame is intended to be a translating motion.
 *
 * @example Frame_translationProbability.txt
 *
 * If either this frame or sinceFrame is an invalid LeapFrame object, then this
 * method returns zero.
 *
 * @param sinceFrame The starting frame for computing the translation.
 * @returns A value between 0 and 1 representing the estimated probability
 * that the overall motion between the current frame and the specified frame
 * is intended to be a translating motion.
 * @available Since 1.0
 */
- (float)translationProbability:(const LeapFrame *)sinceFrame;
/**
 * The axis of rotation derived from the overall rotational motion between
 * the current frame and the specified frame.
 *
 * The returned direction vector is normalized.
 *
 * The Leap derives frame rotation from the relative change in position and
 * orientation of all objects detected in the field of view.
 *
 * @example Frame_rotationAxis.txt
 *
 * If either this frame or sinceFrame is an invalid LeapFrame object, or if no
 * rotation is detected between the two frames, a zero vector is returned.
 *
 * @param sinceFrame The starting frame for computing the relative rotation.
 * @returns A LeapVector containing the normalized direction vector representing the axis of the
 * heuristically determined rotational change between the current frame
 * and that specified in the sinceFrame parameter.
 * @available Since 1.0
 */
- (LeapVector *)rotationAxis:(const LeapFrame *)sinceFrame;
/**
 * The angle of rotation around the rotation axis derived from the overall
 * rotational motion between the current frame and the specified frame.
 *
 * The returned angle is expressed in radians measured clockwise around the
 * rotation axis (using the right-hand rule) between the start and end frames.
 * The value is always between 0 and pi radians (0 and 180 degrees).
 *
 * The Leap derives frame rotation from the relative change in position and
 * orientation of all objects detected in the field of view.
 *
 * @example Frame_rotationAngle.txt
 *
 * If either this frame or sinceFrame is an invalid LeapFrame object, then the
 * angle of rotation is zero.
 *
 * @param sinceFrame The starting frame for computing the relative rotation.
 * @returns A positive value containing the heuristically determined
 * rotational change between the current frame and that specified in the
 * sinceFrame parameter.
 * @available Since 1.0
 */
- (float)rotationAngle:(const LeapFrame *)sinceFrame;
/**
 * The angle of rotation around the specified axis derived from the overall
 * rotational motion between the current frame and the specified frame.
 *
 * The returned angle is expressed in radians measured clockwise around the
 * rotation axis (using the right-hand rule) between the start and end frames.
 * The value is always between -pi and pi radians (-180 and 180 degrees).
 *
 * The Leap derives frame rotation from the relative change in position and
 * orientation of all objects detected in the field of view.
 *
 * @example Frame_rotationAngle_axis.txt
 *
 * If either this frame or sinceFrame is an invalid LeapFrame object, then the
 * angle of rotation is zero.
 *
 * @param sinceFrame The starting frame for computing the relative rotation.
 * @param axis The LeapVector representing the direction of the axis to measure rotation around.
 * @returns A value containing the heuristically determined rotational
 * change between the current frame and that specified in the sinceFrame
 * parameter around the given axis.
 * @available Since 1.0
 */
- (float)rotationAngle:(const LeapFrame *)sinceFrame axis:(const LeapVector *)axis;
/**
 * The transform matrix expressing the rotation derived from the overall
 * rotational motion between the current frame and the specified frame.
 *
 * The Leap derives frame rotation from the relative change in position and
 * orientation of all objects detected in the field of view.
 *
 * @example Frame_rotationMatrix.txt
 *
 * If either this frame or sinceFrame is an invalid LeapFrame object, then this
 * method returns an identity matrix.
 *
 * @param sinceFrame The starting frame for computing the relative rotation.
 * @returns A LeapMatrix containing the heuristically determined
 * rotational change between the current frame and that specified in the
 * sinceFrame parameter.
 * @available Since 1.0
 */
- (LeapMatrix *)rotationMatrix:(const LeapFrame *)sinceFrame;
/**
 * The estimated probability that the overall motion between the current
 * frame and the specified frame is intended to be a rotating motion.
 *
 * @example Frame_rotationProbability.txt
 *
 * If either this frame or sinceFrame is an invalid LeapFrame object, then this
 * method returns zero.
 *
 * @param sinceFrame The starting frame for computing the relative rotation.
 * @returns A value between 0 and 1 representing the estimated probability
 * that the overall motion between the current frame and the specified frame
 * is intended to be a rotating motion.
 * @available Since 1.0
 */
- (float)rotationProbability:(const LeapFrame *)sinceFrame;
/**
 * The scale factor derived from the overall motion between the current frame
 * and the specified frame.
 *
 * The scale factor is always positive. A value of 1.0 indicates no
 * scaling took place. Values between 0.0 and 1.0 indicate contraction
 * and values greater than 1.0 indicate expansion.
 *
 * The Leap derives scaling from the relative inward or outward motion of
 * all objects detected in the field of view (independent of translation
 * and rotation).
 *
 * @example Frame_scale.txt
 *
 * If either this frame or sinceFrame is an invalid LeapFrame object, then this
 * method returns 1.0.
 *
 * @param sinceFrame The starting frame for computing the relative scaling.
 * @returns A positive value representing the heuristically determined
 * scaling change ratio between the current frame and that specified in the
 * sinceFrame parameter.
 * @available Since 1.0
 */
- (float)scaleFactor:(const LeapFrame *)sinceFrame;
/**
 * The estimated probability that the overall motion between the current
 * frame and the specified frame is intended to be a scaling motion.
 *
 * @example Frame_scaleProbability.txt
 *
 * If either this frame or sinceFrame is an invalid LeapFrame object, then this
 * method returns zero.
 *
 * @param sinceFrame The starting frame for computing the relative scaling.
 * @returns A value between 0 and 1 representing the estimated probability
 * that the overall motion between the current frame and the specified frame
 * is intended to be a scaling motion.
 * @available Since 1.0
 */
- (float)scaleProbability:(const LeapFrame *)sinceFrame;
/**
 * Reports whether this LeapFrame instance is valid.
 *
 * @example Frame_isValid.txt
 *
 * A valid LeapFrame is one generated by the LeapController object that contains
 * tracking data for all detected entities. An invalid LeapFrame contains no
 * actual tracking data, but you can call its functions without risk of a
 * null pointer exception. The invalid LeapFrame mechanism makes it more
 * convenient to track individual data across the frame history. For example,
 * you can invoke:
 *
 * @example Frame_Valid_Chain.txt
 *
 * for an arbitrary LeapFrame history value, "n", without first checking whether
 * frame: returned a null object. (You should still check that the
 * returned LeapFinger instance is valid.)
 *
 * @returns YES, if this is a valid LeapFrame object; false otherwise.
 * @available Since 1.0
 */
@property (nonatomic, getter = isValid, readonly)BOOL isValid;
- (BOOL)isValid;
/**
 * The current LeapInteractionBox for the frame. See the LeapInteractionBox class
 * documentation for more details on how this class should be used.
 *
 * @example IBox_get.txt
 *
 * @returns The current LeapInteractionBox object.
 * @available Since 1.0
 */
- (LeapInteractionBox *)interactionBox;
/**
 * The instantaneous framerate.
 *
 * The rate at which the Leap Motion software is providing frames of data
 * (in frames per second). The framerate can fluctuate depending on available computing
 * resources, activity within the device field of view, software tracking settings,
 * and other factors.
 *
 * @example Frame_currentFramesPerSecond.txt
 *
 * @returns An estimate of frames per second of the Leap Motion Controller.
 * @available Since 1.0
 */
@property (nonatomic, getter = currentFramesPerSecond, readonly)float currentFramesPerSecond;
- (float)currentFramesPerSecond;
/**
 * Encodes this Frame object as NSData object.
 *
 * \include Frame_serialize.txt
 *
 * @returns The NSData encoding the data for this frame.
 * @since 2.2.1
 */
- (NSData *)serialize;

/**
 * Decodes an NSData object representing frame data into a LeapFrame object.
 *
 * A LeapController object must be instantiated for this function to succeed, but
 * it does not need to be connected. To extract gestures from the deserialized
 * frame, you must enable the appropriate gestures first.
 *
 * \include Frame_deserialize.txt
 *
 * **Note:** The behavior when calling functions which take
 * another LeapFrame object as a parameter is undefined when either frame has
 * been deserialized. For example, calling ``[frame gestures:sinceFrame]`` on a
 * deserialized frame or with a deserialized frame as parameter (or both)
 * does not necessarily return all gestures that occurred between the two
 * frames. Motion functions, like ``[frame scaleFactor:startFrame]``, are more
 * likely to return reasonable results, but could return anomalous values
 * in some cases.
 *
 * @param serializedFrame An NSData object containing the serialized bytes of a frame.
 *
 * @since 2.2.1
 */
+ (LeapFrame *)deserialize:(NSData *)serializedFrame;
/**
 * Returns an invalid LeapFrame object.
 *
 * @example Frame_invalid.txt
 *
 * @returns The invalid LeapFrame instance.
 * @available Since 1.0
 */
+ (LeapFrame *)invalid;

@end

//////////////////////////////////////////////////////////////////////////
//CONFIG

/**
 * Enumerates the possible data types for configuration values.
 *
 * The [LeapConfig type] property returns an item from the LeapValueType enumeration.
 * @available Since 1.0
 */
typedef enum {
    TYPE_UNKNOWN = 0,
    TYPE_BOOLEAN = 1,
    TYPE_INT32 = 2,
    TYPE_FLOAT = 6,
    TYPE_STRING = 8
} LeapValueType;

/**
 * The LeapConfig class provides access to Leap system configuration information.
 *
 * You can get and set gesture configuration parameters using the LeapConfig object
 * obtained from a connected LeapController object.
 *
 * @example Config_get_config.txt
 *
 * The key strings required to
 * identify a configuration parameter include:
 *
 * \table
 * ====================================  ========== ============= =======
 * Key string                            Value type Default value Units
 * ====================================  ========== ============= =======
 * Gesture.Circle.MinRadius              float      5.0           mm
 * Gesture.Circle.MinArc                 float      1.5 * pi      radians
 * Gesture.Swipe.MinLength               float      150           mm
 * Gesture.Swipe.MinVelocity             float      1000          mm/s
 * Gesture.KeyTap.MinDownVelocity        float      50            mm/s
 * Gesture.KeyTap.HistorySeconds         float      0.1           s
 * Gesture.KeyTap.MinDistance            float      3.0           mm
 * Gesture.ScreenTap.MinForwardVelocity  float      50            mm/s
 * Gesture.ScreenTap.HistorySeconds      float      0.1           s
 * Gesture.ScreenTap.MinDistance         float      5.0           mm
 * ====================================  ========== ============= =======
 * \endtable
 *
 * After setting a configuration value, you must call the [LeapConfig save] method
 * to commit the changes. You can save after the LeapController has connected to
 * the Leap Motion service/daemon. In other words, after the LeapController
 * has dispatched the serviceConnected or connected events or
 * LeapController::isConnected is true. The configuration value changes are not persistent;
 * your application needs to set the values everytime it runs.
 *
 * @see LeapCircleGesture
 * @see LeapKeyTapGesture
 * @see LeapScreenTapGesture
 * @see LeapSwipeGesture
 * @available Since 1.0
 */
@interface LeapConfig : NSObject

/**
 * Reports the natural data type for the value related to the specified key.
 *
 * The supported data types defined by the members of the LeapValueType
 * enumeration:
 *
 * **TYPE_BOOLEAN**
 *
 * **TYPE_INT32**
 *
 * **TYPE_FLOAT**
 *
 * **TYPE_STRING**
 *
 * **TYPE_UNKNOWN**
 *
 * @example Config_type.txt
 *
 * @param key The key for the looking up the value in the configuration dictionary.
 * @returns The native data type of the value, that is, the type that does not
 * require a data conversion.
 * @available Since 1.0
 */
- (LeapValueType)type:(NSString *)key;
/**
 * Gets the boolean representation for the specified key.
 *
 * @example Config_getBool.txt
 *
 * @available Since 1.0
 */
- (BOOL)getBool:(NSString *)key;
/**
 * Sets the boolean representation for the specified key.
 *
 * @example Config_setBool.txt
 *
 * @returns YES on success, NO on failure.
 * @available Since 1.0
 */
- (BOOL)setBool:(NSString *)key value:(BOOL)value;
/**
 * Gets the 32-bit integer representation for the specified key.
 *
 * @example Config_getInt32.txt
 *
 * @available Since 1.0
 */
- (int32_t)getInt32:(NSString *)key;
/**
 * Sets the 32-bit integer representation for the specified key.
 *
 * @example Config_setInt32.txt
 *
 * @returns YES on success, NO on failure.
 * @available Since 1.0
 */
- (BOOL)setInt32:(NSString *)key value:(int32_t)value;
/**
 * Gets the floating point representation for the specified key.
 *
 * @example Config_getFloat.txt
 *
 * @available Since 1.0
 */
- (float)getFloat:(NSString *)key;
/**
 * Sets the floating point representation for the specified key.
 *
 * @example Config_setFloat.txt
 *
 * @returns YES on success, NO on failure.
 * @available Since 1.0
 */
- (BOOL)setFloat:(NSString *)key value:(float)value;
/**
 * Gets the NSString representation for the specified key.
 *
 * @example Config_getString.txt
 *
 * @available Since 1.0
 */
- (NSString *)getString:(NSString *)key;
/**
 * Sets the string representation for the specified key.
 *
 * @example Config_setString.txt
 *
 * @returns YES on success, NO on failure.
 * @available Since 1.0
 */
- (BOOL)setString:(NSString *)key value:(NSString *)value;
/**
 * Saves the current state of the config.
 *
 * Call [LeapConfig save:] after making a set of configuration changes. The
 * [LeapConfig save:] function transfers the configuration changes to the Leap
 * application. You can save after the LeapController has connected to
 * the Leap Motion service/daemon. In other words, after the LeapController
 * has dispatched the serviceConnected or connected events or
 * LeapController::isConnected is true. The configuration value changes are not persistent; your
 * application needs to set the values every time it runs.
 *
 * @example Config_save.txt
 *
 * @returns TRUE on success, NO on failure.
 * @available Since 1.0
 */
- (BOOL)save;

@end

//////////////////////////////////////////////////////////////////////////
//CONTROLLER
/**
 * The LeapController class is your main interface to the Leap Motion Controller
 * hardware.
 *
 * Create an instance of this LeapController class to access frames of tracking
 * data and configuration information. Frame data can be polled at any time
 * using the [LeapController frame:] function. Set the `frame:` parameter to 0
 * to get the most recent frame. Set the parameter to a positive integer
 * to access previous frames. For example, `[controller frame:10]` returns the
 * frame that occurred ten frames ago. A controller stores up to 60 frames in its
 * frame history.
 *
 * Polling is an appropriate strategy for applications which already have an
 * intrinsic update loop, such as a game. You can also add a listener object
 * or delegate to the controller to handle events as they occur.
 * The Leap dispatches events to the listener upon initialization and exiting,
 * on connection changes, and when a new frame of tracking data is available.
 * When these events occur, the controller object invokes the appropriate
 * callback function.
 *
 * **Polling**
 *
 * Create an instance of the LeapController class using the default initializer:
 *
 * @example Controller_constructor.txt
 *
 * Access the frame data at regular intervals:
 *
 * @example Controller_frame.txt
 *
 * You can check [LeapController isConnected] to determine if the controller
 * is connected to the Leap software.
 *
 * **LeapListener protocol**
 *
 * Implement a class adopting the LeapListener protocol.
 *
 * Create an instance of the LeapController class and assign your LeapListener object to it:
 *
 * @example Controller_withListener.txt
 *
 * The controller subscribes the LeapListener instance to the appropriate NSNotifications
 * for the Leap events. When a new frame of data is ready, the controller dispatches an
 * NSNotification on the main application thread, which is handled by your
 * [LeapListener onFrame:] implementation.
 *
 * **LeapDelegate protocol**
 *
 * Implement a class adopting the LeapDelegate protocol.
 *
 * Create an instance of the LeapController class and assign your LeapListener object to it:
 *
 * @example Controller_addDelegate.txt
 *
 * When a new frame of data is ready, the controller calls the
 * [LeapDelegate onFrame:] method. The Controller object is multithreaded and
 * calls the LeapDelegate functions on its own thread, not on an application thread.
 *
 *
 * You can handle the other Leap events, `onInit`, `onConnect`, `onDisconnect`,
 * `onServiceConnect`, `onServiceDisconnect`, `onDeviceChange`, and `onExit` in
 * the same manner.
 *
 * @available Since 1.0
 */
@interface LeapController : NSObject

/**
 * Initializes a LeapController instance.
 *
 * @example Controller_constructor.txt
 *
 * @available Since 1.0
 */
- (id)init;
/**
 * Initializes a LeapController instance and assigns a listener.
 *
 * @example Controller_withListener.txt
 *
 * * *Note:* You can use either a listener or a delegate, but not both.
 *
 * @param listener An object adopting the LeapListener protocol.
 * @available Since 1.0
 */
- (id)initWithListener:(id)listener;
/**
 * This method has been deprecated. Use [LeapController isPolicySet:] instead.
 *
 * @returns The current policy flags.
 * @deprecated 2.1.6
 */
- (LeapPolicyFlag)policyFlags;
/**
 * This method has been deprecated. Use [LeapController setPolicy:] and
 * [LeapController clearPolicy:] instead.
 *
 * @returns The current policy flags.
 * @deprecated 2.1.6
 */
- (void)setPolicyFlags:(LeapPolicyFlag)flags;
/**
 * Requests setting a policy.
 *
 * A request to change a policy is subject to user approval and a policy
 * can be changed by the user at any time (using the Leap settings window).
 * The desired policy flags must be set every time an application runs.
 *
 * Policy changes are completed asynchronously and, because they are subject
 * to user approval, may not complete successfully. Call
 * [LeapController isPolicySet:] after a suitable interval to test whether
 * the change was accepted.
 *
 * The current policies are:
 *
 * **LEAP_POLICY_BACKGROUND_FRAMES** -- requests that your application receives frames
 *   when it is not the foreground application for user input.\n
 *
 *   The background frames policy determines whether an application
 *   receives frames of tracking data while in the background. By
 *   default, the Leap Motion  software only sends tracking data to the foreground application.
 *   Only applications that need this ability should request the background
 *   frames policy. The "Allow Background Apps" checkbox must be enabled in the
 *   Leap Motion Control Panel or this policy will be denied.
 *
 * **LEAP_POLICY_IMAGES** -- request that your application receives images from the
 *   device cameras. The "Allow Images" checkbox must be enabled in the
 *   Leap Motion Control Panel or this policy will be denied.\n
 *   The images policy determines whether an application recieves image data from
 *   the Leap Motion sensors which each frame of data. By default, this data is
 *   not sent. Only applications that use the image data should request this policy.\n
 *
 * **LEAP_POLICY_OPTIMIZE_HMD** -- request that the tracking be optimized for head-mounted
 *   tracking.\n
 *   The optimize HMD policy improves tracking in situations where the Leap
 *   Motion hardware is attached to a head-mounted display. This policy is
 *   not granted for devices that cannot be mounted to an HMD, such as
 *   Leap Motion controllers embedded in a laptop or keyboard.
 *
 * @example Controller_setPolicy.txt
 *
 * @param flags A PolicyFlag value indicating the policy to request. Must be
 * a member of the LeapPolicyFlags enumeration:
 *
 * **LEAP_POLICY_BACKGROUND_FRAMES**
 *
 * **LEAP_POLICY_IMAGES**
 *
 * **LEAP_POLICY_OPTIMIZE_HMD**
 *
 * @available Since 2.1.6
 */
- (void)setPolicy:(LeapPolicyFlag)policy;
/**
 * Requests clearing a policy.
 *
 * Policy changes are completed asynchronously and, because they are subject
 * to user approval, may not complete successfully. Call
 * [LeapController isPolicySet:] after a suitable interval to test whether
 * the change was accepted.
 *
 * @example Controller_clearPolicy.txt
 *
 * @param flags A PolicyFlag value indicating the policy to request.
 * @available Since 2.1.6
 */
- (void)clearPolicy:(LeapPolicyFlag)policy;
/**
 * Gets the active setting for a specific policy.
 *
 * Use this function to determine the current policy state.
 * Keep in mind that setting a policy flag is asynchronous, so changes are
 * not effective immediately after calling [LeapController setPolicy:]. In addition, a
 * policy request can be declined by the user. You should always set the
 * policy flags required by your application at startup and check that the
 * policy change request was successful after an appropriate interval.
 *
 * If the controller object is not connected to the Leap, then the default
 * policy state is returned.
 *
 * @example Controller_isPolicySet.txt
 * This method has been deprecated. Use [LeapController isPolicySet:] instead.
 *
 * @param flags A PolicyFlag value indicating the policy to query.
 * @returns A boolean indicating whether the specified policy has been set.
 * @available Since 2.1.6
 */
- (BOOL)isPolicySet:(LeapPolicyFlag)policy;
/**
 * Adds a listener to this LeapController.
 *
 * When you add an object adopting the LeapListener protocol to a LeapController,
 * the controller automatically subscribes the listener to NSNotifications
 * dispatched for the Leap events.
 *
 * @example Controller_addListener.txt
 *
 * *Note:* You cannot add a listener when there is already a delegate assigned.
 *
 * @param listener An object adopting the LeapListener protocol.
 * @returns BOOL Whether or not the listener was successfully added to the list
 * of listeners.
 * @available Since 1.0
 */
- (BOOL)addListener:(id)listener;
/**
 * Unsubscribes the listener object from receiving Leap NSNotifications.
 *
 * @example Controller_removeListener.txt
 *
 * @param listener The listener to unsubscribe.
 * @returns BOOL Whether or not the listener was successfully removed.
 * @available Since 1.0
 */
- (BOOL)removeListener:(id)listener;
/**
 * Initializes a LeapController instance and assigns a delegate.
 *
 * @example Controller_withDelegate.txt
 *
 * * *Note:* You can use either a delegate or a listener, but not both.
 *
 * @param delegate An object adopting the LeapDelegate protocol.
 * @available Since 1.0
 */
- (id)initWithDelegate:(id)delegate;
/**
 * Adds a delegate to this LeapController.
 *
 * @example Controller_addDelegate.txt
 *
 * *Note:* You cannot add a delegate when there is already a listener assigned.
 *
 * @param delegate An object adopting the LeapDelegate protocol.
 * @returns BOOL Whether or not the delegate was successfully added.
 * @available Since 1.0
 */
- (BOOL)addDelegate:(id)delegate;
/**
 * Removes the delegate assigned to this LeapController.
 *
 * @example Controller_removeDelegate.txt
 *
 * @returns BOOL Whether or not the delegate was successfully removed.
 * @available Since 1.0
 */
- (BOOL)removeDelegate;
/**
 * Returns a LeapFrame containing a frame of tracking data from the Leap. Use the optional
 * history parameter to specify which frame to retrieve. Call
 * `[controller frame:0]` to access the most recent frame; call
 * `[controller frame:1]` to access the previous frame, and so on. If you use a
 * history value greater than the number of stored frames, then the controller
 * returns an invalid frame.
 *
 * @example Controller_frame.txt
 *
 * @param history The age of the LeapFrame to return, counting backwards from
 * the most recent frame (0) into the past and up to the maximum age (59).
 * @returns The specified LeapFrame; or, if no history parameter is specified,
 * the newest frame. If a frame is not available at the specified history
 * position, an invalid LeapFrame is returned.
 * @available Since 1.0
 */
- (LeapFrame *)frame:(int)history;
/**
 * Returns a Config object, which you can use to query the Leap system for
 * configuration information.
 *
 * @example Controller_config.txt
 *
 * @available Since 1.0
 */
@property (nonatomic, getter = config, readonly)LeapConfig *config;
- (LeapConfig *)config;
/**
 * Returns a list of currently attached devices.  Devices can be queried for
 * device-specific information such as field of view.
 *
 * Currently the Leap controller only recognizes a single device at a time.
 *
 * @example Controller_devices.txt
 *
 * @available Since 1.0
 */
@property (nonatomic, getter = devices, readonly)NSArray *devices;
- (NSArray *)devices;
/**
 * Reports whether this LeapController is connected to the Leap device.
 *
 * When you first create a LeapController object, isConnected returns false.
 * After the controller finishes initializing and connects to the Leap,
 * isConnected will return true.
 *
 * You can either handle the onConnect event using a LeapListener or LeapDelegate
 * instance or poll the isConnected function if you need to wait for your
 * application to be connected to the Leap before performing some other
 * operation.
 *
 * @example Controller_isConnected.txt
 *
 * @returns True, if connected; false otherwise.
 * @available Since 1.0
 */
@property (nonatomic, getter = isConnected, readonly)BOOL isConnected;
- (BOOL)isConnected;
/**
 * Reports whether this LeapService is in communication with the client API.
 *
 * When you first create a LeapController object, isServiceConnected returns false.
 * After the controller finishes initializing and makes contact with the LeapService,
 * isServiceConnected will return true.
 *
 * You can either handle the onServiceConnect event using a LeapListener or LeapDelegate
 * instance or poll the isServiceConnected function if you need to wait for your
 * application to be connected to the Leap Service before performing some other
 * operation.
 *
 * @example Controller_isServiceConnected.txt
 *
 * @returns True, if connected; false otherwise.
 * @available Since 1.2
 */
@property (nonatomic, getter = isServiceConnected, readonly)BOOL isServiceConnected;
- (BOOL)isServiceConnected;
/**
 * Reports whether this application is the focused, foreground application.
 *
 * Only the foreground application receives tracking information from
 * the Leap Motion Controller.
 *
 * @example Controller_hasFocus.txt
 *
 * @returns True, if application has focus; false otherwise.
 * @available Since 1.0
 */
@property (nonatomic, getter = hasFocus, readonly)BOOL hasFocus;
- (BOOL)hasFocus;
/**
 * Enables or disables reporting of a specified gesture type.
 *
 * By default, all gesture types are disabled. When disabled, gestures of the
 * disabled type are never reported and will not appear in the frame
 * gesture list.
 *
 * @example Controller_hasFocus.txt
 *
 * As a performance optimization, only enable recognition for the types
 * of movements that you use in your application.
 *
 * @param gestureType The type of gesture to enable or disable. Must be a
 * member of the LeapGestureType enumeration:
 *
 * **LEAP_GESTURE_TYPE_SWIPE**  -- A straight line movement by the hand with fingers extended.
 *
 * **LEAP_GESTURE_TYPE_CIRCLE** -- A circular movement by a finger.
 *
 * **LEAP_GESTURE_TYPE_SCREEN_TAP** -- A forward tapping movement by a finger.
 *
 * **LEAP_GESTURE_TYPE_KEY_TAP** -- A downward tapping movement by a finger.
 *
 * @param enable YES, to enable the specified gesture type; NO,
 * to disable.
 * @see [LeapController isGestureEnabled:]
 * @available Since 1.0
 */
- (void)enableGesture:(LeapGestureType)gestureType enable:(BOOL)enable;
/**
 * Reports whether the specified gesture type is enabled.
 *
 * @example Controller_isGestureEnabled.txt
 *
 * @param gestureType The type of gesture to check.  Must be a
 * member of the LeapGestureType enumeration:
 *
 * **LEAP_GESTURE_TYPE_SWIPE**  -- A straight line movement by the hand with fingers extended.
 *
 * **LEAP_GESTURE_TYPE_CIRCLE** -- A circular movement by a finger.
 *
 * **LEAP_GESTURE_TYPE_SCREEN_TAP** -- A forward tapping movement by a finger.
 *
 * **LEAP_GESTURE_TYPE_KEY_TAP** -- A downward tapping movement by a finger.
 *
 * @return YES, if the specified type is enabled; NO, otherwise.
 * @see [LeapController enableGesture:enable:]
 * @available Since 1.0
 */
- (BOOL)isGestureEnabled:(LeapGestureType)gestureType;
/*
 * Deprecated as of version 1.2
 */
@property (nonatomic, getter = locatedScreens, readonly)NSArray *locatedScreens;
- (NSArray *)locatedScreens;
/**
 * The most recent set of images from the Leap Motion cameras.
 *
 * Depending on timing and the current processing frame rate, the images
 * obtained with this function can be newer than images obtained from
 * the current frame of tracking data.
 *
 * @example Controller_images.txt
 *
 * @return An NSArray object containing the most recent camera images.
 * @since 2.2.1
 */
@property (nonatomic, getter = images, readonly)NSArray *images;
- (NSArray *)images;
/**
 * Returns a timestamp value as close as possible to the current time.
 * Values are in microseconds, as with all the other timestamp values.
 *
 * @example Controller_now.txt
 *
 * @since 2.3.0
 **/
- (int64_t)now;
@end

//////////////////////////////////////////////////////////////////////////
//LISTENER
/**
 * The LeapListener protocol defines a set of methods that you can
 * implement to respond to NSNotification messages dispatched by a LeapController object.
 *
 * To use the LeapListener protocol, implement a class adopting the protocol
 * and assign an instance of that class to a LeapController instance:
 *
 *     MYListener *listener = [[MYListener alloc] init];
 *     LeapController *controller = [[LeapController alloc] initWithListener:listener];
 *
 * The controller subscribes the LeapListener instance to the appropriate NSNotifications
 * for the Leap events. When a new frame of data is ready, the controller dispatches an
 * NSNotification on the main application thread, which is handled by your
 * [LeapListener onFrame:] implementation.
 *
 * You can handle the other Leap events, `onInit`, `onConnect`, `onDisconnect`,
 * `onServiceConnect`, `onServiceDisconnect`, `onDeviceChange`, `onExit`, `onFocusGained`,
 * and `onFocusLost` in the same manner.
 *
 * You must have a running NSRunLoop to receive NSNotification objects.
 * This is usually present and running by default in a Cocoa application.
 * Calling [LeapController addListener:] takes care subscribing the listener object
 * to the appropriate notifications. The LeapListener object is the notification observer,
 * while the LeapController object is the notification sender. You can also subscribe to
 * notifications manually. For example, to subscribe to the OnFrame message, call:
 *
 *     [[NSNotificationCenter defaultCenter] selector:@selector(onFrame:) name:@"OnFrame" object:controller]]
 *
 * However, at least one listener must be added to the controller with [LeapController addListener:]
 * or the controller does not bother to dispatch notification messages.
 *
 * Using the LeapListener protocol is not mandatory. You can also use
 * a delegate implementing the LeapDelegate protocol or simply poll the
 * controller object (as described in the LeapController class overview).
 * @available Since 1.0
 */
@protocol LeapListener<NSObject>

@optional
/**
 * Dispatched once, when the LeapController has finished initializing.
 *
 * Only the first LeapListener added to the controller receives this notification.
 *
 *     - (void)onInit:(NSNotification *)notification
 *     {
 *         NSLog(@"Initialized");
 *         //...
 *     }
 *
 * @param notification The LeapController object dispatching the notification.
 * @available Since 1.0
 */
- (void)onInit:(NSNotification *)notification;
/**
 * Called when this application becomes the foreground application.
 *
 * Only the foreground application receives tracking data from the Leap
 * Motion Controller. This function is only called when the controller
 * object is in a connected state.
 *
 *     - (void)onFocusGained:(NSNotification *)notification
 *     {
 *         NSLog(@"Focus Gained");
 *     }
 *
 * @param notification The LeapController object dispatching the notification.
 * @available Since 1.0
 */
- (void)onFocusGained:(NSNotification *)notification;
/**
 * Called when this application loses the foreground focus.
 *
 * Only the foreground application receives tracking data from the Leap
 * Motion Controller. This function is only called when the controller
 * object is in a connected state.
 *
 *     - (void)onFocusLost:(NSNotification *)notification
 *     {
 *         NSLog(@"Focus Lost");
 *     }
 *
 * @param controller The parent LeapController object.
 * @available Since 1.0
 */
- (void)onFocusLost:(NSNotification *)notification;

/**
 * Dispatched when the LeapController object connects to the Leap software, or when
 * this ListenerListener object is added to a controller that is already connected.
 *
 *     - (void)onConnect:(NSNotification *)notification
 *     {
 *        NSLog(@"Connected");
 *         LeapController *aController = (LeapController *)[notification object];
 *         [aController enableGesture:LEAP_GESTURE_TYPE_CIRCLE enable:YES];
 *         //...
 *     }
 *
 * @param notification The LeapController object dispatching the notification.
 * @available Since 1.0
 */
- (void)onConnect:(NSNotification *)notification;
/**
 * Dispatched when the LeapController object disconnects from the Leap software.
 * The controller can disconnect when the Leap device is unplugged, the
 * user shuts the Leap software down, or the Leap software encounters an
 * unrecoverable error.
 *
 *     - (void)onDisconnect:(NSNotification *)notification
 *     {
 *         NSLog(@"Disconnected");
 *     }
 *
 * Note: When you launch a Leap-enabled application in a debugger, the
 * Leap library does not disconnect from the application. This is to allow
 * you to step through code without losing the connection because of time outs.
 *
 * @param notification The LeapController object dispatching the notification.
 * @available Since 1.0
 */
- (void)onDisconnect:(NSNotification *)notification;
/**
 * Dispatched when the LeapController object connects to the Leap software, or when
 * this ListenerListener object is added to a controller that is already connected.
 *
 *     - (void)onServiceConnect:(NSNotification *)notification
 *     {
 *         NSLog(@"Service Connected");
 *         LeapController *aController = (LeapController *)[notification object];
 *         [aController enableGesture:LEAP_GESTURE_TYPE_CIRCLE enable:YES];
 *         //...
 *     }
 *
 * @param notification The LeapController object dispatching the notification.
 * @available Since 1.0
 */
- (void)onServiceConnect:(NSNotification *)notification;
/**
 * Dispatched when the LeapController object disconnects from the Leap software.
 * The controller can disconnect when the Leap device is unplugged, the
 * user shuts the Leap software down, or the Leap software encounters an
 * unrecoverable error.
 *
 *     - (void)onServiceDisconnect:(NSNotification *)notification
 *     {
 *         NSLog(@"Service Disconnected");
 *     }
 *
 * Note: When you launch a Leap-enabled application in a debugger, the
 * Leap library does not disconnect from the application. This is to allow
 * you to step through code without losing the connection because of time outs.
 *
 * @param notification The LeapController object dispatching the notification.
 * @available Since 1.0
 */
- (void)onServiceDisconnect:(NSNotification *)notification;
/**
 * Called when a Leap Motion controller plugged in, unplugged, or the device changes state.
 *
 * State changes include changes in frame rate and entering or leaving "robust" mode.
 * Note that there is currently no way to query whether a device is in robust mode.
 * You can use Frame::currentFramerate() to get the framerate.
 *
 *     - (void)onDeviceChange:(NSNotification *)notification
 *     {
 *         NSLog(@"Device Changed");
 *     }
 *
 * @param notification The LeapController object dispatching the notification.
 * @since 1.2
 */
- (void)onDeviceChange:(NSNotification *)notification;

/**
 * Dispatched when this LeapListener object is removed from the LeapController
 * or the controller instance is destroyed.
 *
 *     - (void)onExit:(NSNotification *)notification
 *     {
 *         NSLog(@"Exited");
 *     }
 *
 * @param notification The LeapController object dispatching the notification.
 * @available Since 1.0
 */
- (void)onExit:(NSNotification *)notification;
/**
 * Dispatched when a new LeapFrame containing hand and finger tracking data is available.
 * Access the new frame data using the [LeapController frame:] function.
 *
 *     - (void)onFrame:(NSNotification *)notification
 *     {
 *          NSLog(@"New LeapFrame");
 *          LeapController *controller = (LeapController *)[notification object];
 *          LeapFrame *frame = [controller frame:0];
 *          //...
 *     }
 *
 * Note, the LeapController skips any pending onFrame notifications while your
 * onFrame handler executes. If your implementation takes too long to return,
 * one or more frames can be skipped. The controller still inserts the skipped
 * frames into the frame history. You can access recent frames by setting
 * the history parameter when calling the [LeapController frame:] function.
 * You can determine if any pending onFrame events were skipped by comparing
 * the ID of the most recent frame with the ID of the last received frame.
 *
 * @param notification The LeapController object dispatching the notification.
 * @available Since 1.0
 */
- (void)onFrame:(NSNotification *)notification;
/**
 * Called when new images are available.
 * Access the NSArray containing the new images using the ``[LeapController images]`` function.
 *
 *     - (void)onImages:(NSNotification *)notification
 *     {
 *          NSLog(@"New image list");
 *          LeapController *controller = (LeapController *)[notification object];
 *          NSArray *images = [controller images];
 *          //...
 *     }
 *
 * @since 2.2.1
 */
- (void)onImages:(NSNotification *)notification;
@end

//////////////////////////////////////////////////////////////////////////
//DELEGATE
/**
 * The LeapDelegate protocol defines a set of methods that you can
 * implement in a delegate object for a LeapController. The
 * LeapController calls the delegate methods when Leap events occur,
 * such as when a new frame of data is available.
 *
 * To use the LeapDelegate protocol, implement a class adopting the LeapDelegate
 * protocol and assign it to a LeapController instance:
 *
 *     MYDelegate *delegate = [[MYDelegate alloc] init];
 *     LeapController *controller = [[LeapController alloc] init];
 *     [controller addDelegate:delegate];
 *
 * When a new frame of data is ready, the controller calls the
 * [LeapDelegate onFrame:] method. The other Leap events, `onInit`, `onConnect`,
 * `onDisconnect`, `onServiceConnect`, `onServiceDisconnect`, `onExit`,
 * `onFocusGained`, and `onFocusLost` are handled in the same manner. The
 * Controller object is multithreaded and calls the LeapDelegate functions on
 * its own threads, not on an application thread.
 *
 * Using the LeapDelegate protocol is not mandatory. You can also use
 * NSNotifications with a LeapListener object or simply poll the
 * controller object (as described in the LeapController class overview).
 * @available Since 1.0
 */
@protocol LeapDelegate<NSObject>

@optional
/**
 * Called once, when the LeapController has finished initializing.
 *
 *
 *     - (void)onInit:(LeapController *)controller
 *     {
 *         NSLog(@"Initialized");
 *         //...
 *     }
 *
 * @param controller The parent LeapController object.
 * @available Since 1.0
 */
- (void)onInit:(LeapController *)controller;
/**
 * Called when the LeapController object connects to the Leap software, or when
 * this ListenerDelegate object is added to a controller that is already connected.
 *
 *     - (void)onConnect:(LeapController *)controller
 *     {
 *         NSLog(@"Connected");
 *         [controller enableGesture:LEAP_GESTURE_TYPE_CIRCLE enable:YES];
 *         //...
 *     }
 *
 * @param controller The parent LeapController object.
 * @available Since 1.0
 */
- (void)onConnect:(LeapController *)controller;
/**
 * Called when the LeapController object disconnects from the Leap software.
 * The controller can disconnect when the Leap device is unplugged, the
 * user shuts the Leap software down, or the Leap software encounters an
 * unrecoverable error.
 *
 *     - (void)onDisconnect:(LeapController *)controller
 *     {
 *         NSLog(@"Disconnected");
 *     }
 *
 * @param controller The parent LeapController object.
 * @available Since 1.0
 */
- (void)onDisconnect:(LeapController *)controller;
/**
 * Called when this LeapDelegate object is removed from the LeapController
 * or the controller instance is destroyed.
 *
 *     - (void)onExit:(LeapController *)controller
 *     {
 *         NSLog(@"Exited");
 *     }
 *
 * Note: When you launch a Leap-enabled application in a debugger, the
 * Leap library does not disconnect from the application. This is to allow
 * you to step through code without losing the connection because of time outs.
 *
 * @param controller The parent LeapController object.
 * @available Since 1.0
 */
- (void)onExit:(LeapController *)controller;
/**
 * Called when a new frame of hand and finger tracking data is available.
 * Access the new frame data using the [LeapController frame:] function.
 *
 *     - (void)onFrame:(LeapController *)controller
 *     {
 *          NSLog(@"New LeapFrame");
 *          LeapFrame *frame = [controller frame:0];
 *          //...
 *     }
 *
 * Note, the LeapController skips any pending frames while your
 * onFrame handler executes. If your implementation takes too long to return,
 * one or more frames can be skipped. The controller still inserts the skipped
 * frames into the frame history. You can access recent frames by setting
 * the history parameter when calling the [LeapController frame:] function.
 * You can determine if any pending frames were skipped by comparing
 * the ID of the current frame with the ID of the previous received frame.
 *
 * @param controller The parent LeapController object.
 * @available Since 1.0
 */
- (void)onFrame:(LeapController *)controller;
/**
 * Called when this application becomes the foreground application.
 *
 * Only the foreground application receives tracking data from the Leap
 * Motion Controller. This function is only called when the controller
 * object is in a connected state.
 *
 *     - (void)onFocusGained:(LeapController *)controller
 *     {
 *         NSLog(@"Focus Gained");
 *     }
 *
 * @param controller The parent LeapController object.
 * @available Since 1.0
 */
- (void)onFocusGained:(LeapController *)controller;
/**
 * Called when this application loses the foreground focus.
 *
 * Only the foreground application receives tracking data from the Leap
 * Motion Controller. This function is only called when the controller
 * object is in a connected state.
 *
 *     - (void)onFocusLost:(LeapController *)controller
 *     {
 *         NSLog(@"Focus Lost");
 *     }
 *
 * @param controller The parent LeapController object.
 * @available Since 1.0
 */
- (void)onFocusLost:(LeapController *)controller;
/**
 * Called if the Leap Motion daemon/service disconnects from your application Controller.
 *
 * Normally, this callback is not invoked. It is only called if some external event
 * or problem shuts down the service or otherwise interrupts the connection.
 *
 *     - (void)onServiceConnect:(LeapController *)controller
 *     {
 *         NSLog(@"Service Connected");
 *     }
 *
 * @param controller The Controller object invoking this callback function.
 * @available Since 1.2
 */
- (void)onServiceConnect:(LeapController *)controller;

/**
 * Called if the Leap Motion daemon/service disconnects from your application Controller.
 *
 * Normally, this callback is not invoked. It is only called if some external event
 * or problem shuts down the service or otherwise interrupts the connection.
 *
 *     - (void)onServiceDisconnect:(LeapController *)controller
 *     {
 *         NSLog(@"Service Disconnected");
 *     }
 *
 * @param controller The Controller object invoking this callback function.
 * @available Since 1.2
 */
- (void)onServiceDisconnect:(LeapController *)controller;
/**
 * Called when a Leap Motion controller plugged in, unplugged, or the device changes state.
 *
 * State changes include changes in frame rate and entering or leaving "robust" mode.
 * Note that there is currently no way to query whether a device is in robust mode.
 * You can use Frame::currentFramerate() to get the framerate.
 *
 *     - (void)onDeviceChange:(LeapController *)controller
 *     {
 *         NSLog(@"Device Changed");
 *     }
 *
 * @param controller The Controller object invoking this callback function.
 * @since 1.2
 */
- (void)onDeviceChange:(LeapController *)controller;
/**
 * Called when new images are available.
 * Access the NSArray containing the new images using the ``[LeapController images]`` function.
 *
 *     - (void)onImages:(LeapController *)controller
 *     {
 *          NSLog(@"New image list");
 *          NSArray *images = [controller images];
 *          //...
 *     }
 *
 * @param controller The Controller object invoking this callback function.
 * @since 2.2.1
 */
- (void)onImages:(LeapController *)controller;
@end



//////////////////////////////////////////////////////////////////////////
