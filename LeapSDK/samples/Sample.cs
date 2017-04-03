/******************************************************************************\
* Copyright (C) 2012-2014 Leap Motion, Inc. All rights reserved.               *
* Leap Motion proprietary and confidential. Not for distribution.              *
* Use subject to the terms of the Leap Motion SDK Agreement available at       *
* https://developer.leapmotion.com/sdk_agreement, or another agreement         *
* between Leap Motion and you, your company or other organization.             *
\******************************************************************************/
using System;
using System.Threading;
using Leap;

class SampleListener : Listener
{
	private Object thisLock = new Object ();

	private void SafeWriteLine (String line)
	{
		lock (thisLock) {
			Console.WriteLine (line);
		}
	}

	public override void OnInit (Controller controller)
	{
		SafeWriteLine ("Initialized");
	}

	public override void OnConnect (Controller controller)
	{
		SafeWriteLine ("Connected");
		controller.EnableGesture (Gesture.GestureType.TYPE_CIRCLE);
		controller.EnableGesture (Gesture.GestureType.TYPE_KEY_TAP);
		controller.EnableGesture (Gesture.GestureType.TYPE_SCREEN_TAP);
		controller.EnableGesture (Gesture.GestureType.TYPE_SWIPE);
	}

	public override void OnDisconnect (Controller controller)
	{
        //Note: not dispatched when running in a debugger.
		SafeWriteLine ("Disconnected");
	}

	public override void OnExit (Controller controller)
	{
		SafeWriteLine ("Exited");
	}

	public override void OnFrame (Controller controller)
	{
		// Get the most recent frame and report some basic information
		Frame frame = controller.Frame ();

		SafeWriteLine ("Frame id: " + frame.Id
                    + ", timestamp: " + frame.Timestamp
                    + ", hands: " + frame.Hands.Count
                    + ", fingers: " + frame.Fingers.Count
                    + ", tools: " + frame.Tools.Count
                    + ", gestures: " + frame.Gestures ().Count);

		foreach (Hand hand in frame.Hands) {
			SafeWriteLine ("  Hand id: " + hand.Id
						+ ", palm position: " + hand.PalmPosition);
			// Get the hand's normal vector and direction
			Vector normal = hand.PalmNormal;
			Vector direction = hand.Direction;

			// Calculate the hand's pitch, roll, and yaw angles
			SafeWriteLine ("  Hand pitch: " + direction.Pitch * 180.0f / (float)Math.PI + " degrees, "
                        + "roll: " + normal.Roll * 180.0f / (float)Math.PI + " degrees, "
                        + "yaw: " + direction.Yaw * 180.0f / (float)Math.PI + " degrees");

			// Get the Arm bone
            Arm arm = hand.Arm;
			SafeWriteLine ("  Arm direction: " + arm.Direction
                        + ", wrist position: " + arm.WristPosition
                        + ", elbow position: " + arm.ElbowPosition);

			// Get fingers
			foreach (Finger finger in hand.Fingers) {
				SafeWriteLine ("    Finger id: " + finger.Id
					        + ", " + finger.Type.ToString()
					        + ", length: " + finger.Length
					        + "mm, width: " + finger.Width + "mm");

				// Get finger bones
				Bone bone;
				foreach (Bone.BoneType boneType in (Bone.BoneType[]) Enum.GetValues(typeof(Bone.BoneType)))
				{
					bone = finger.Bone(boneType);
					SafeWriteLine("      Bone: " + boneType
						        + ", start: " + bone.PrevJoint
						        + ", end: " + bone.NextJoint
						        + ", direction: " + bone.Direction);
				}
			}

		}

		// Get tools
		foreach (Tool tool in frame.Tools) {
			SafeWriteLine ("  Tool id: " + tool.Id
				        + ", position: " + tool.TipPosition
				        + ", direction " + tool.Direction);
		}

		// Get gestures
		GestureList gestures = frame.Gestures ();
		for (int i = 0; i < gestures.Count; i++) {
			Gesture gesture = gestures [i];

			switch (gesture.Type) {
			case Gesture.GestureType.TYPE_CIRCLE:
				CircleGesture circle = new CircleGesture (gesture);

                // Calculate clock direction using the angle between circle normal and pointable
				String clockwiseness;
				if (circle.Pointable.Direction.AngleTo (circle.Normal) <= Math.PI / 2) {
					//Clockwise if angle is less than 90 degrees
					clockwiseness = "clockwise";
				} else {
					clockwiseness = "counterclockwise";
				}

				float sweptAngle = 0;

                // Calculate angle swept since last frame
				if (circle.State != Gesture.GestureState.STATE_START) {
					CircleGesture previousUpdate = new CircleGesture (controller.Frame (1).Gesture (circle.Id));
					sweptAngle = (circle.Progress - previousUpdate.Progress) * 360;
				}

				SafeWriteLine ("  Circle id: " + circle.Id
                               + ", " + circle.State
                               + ", progress: " + circle.Progress
                               + ", radius: " + circle.Radius
                               + ", angle: " + sweptAngle
                               + ", " + clockwiseness);
				break;
			case Gesture.GestureType.TYPE_SWIPE:
				SwipeGesture swipe = new SwipeGesture (gesture);
				SafeWriteLine ("  Swipe id: " + swipe.Id
                               + ", " + swipe.State
                               + ", position: " + swipe.Position
                               + ", direction: " + swipe.Direction
                               + ", speed: " + swipe.Speed);
				break;
			case Gesture.GestureType.TYPE_KEY_TAP:
				KeyTapGesture keytap = new KeyTapGesture (gesture);
				SafeWriteLine ("  Tap id: " + keytap.Id
                               + ", " + keytap.State
                               + ", position: " + keytap.Position
                               + ", direction: " + keytap.Direction);
				break;
			case Gesture.GestureType.TYPE_SCREEN_TAP:
				ScreenTapGesture screentap = new ScreenTapGesture (gesture);
				SafeWriteLine ("  Tap id: " + screentap.Id
                               + ", " + screentap.State
                               + ", position: " + screentap.Position
                               + ", direction: " + screentap.Direction);
				break;
			default:
				SafeWriteLine ("  Unknown gesture type.");
				break;
			}
		}

		if (!frame.Hands.IsEmpty || !frame.Gestures ().IsEmpty) {
			SafeWriteLine ("");
		}
	}
}

class Sample
{
	public static void Main ()
	{
		// Create a sample listener and controller
		SampleListener listener = new SampleListener ();
		Controller controller = new Controller ();

		// Have the sample listener receive events from the controller
		controller.AddListener (listener);

		// Keep this process running until Enter is pressed
		Console.WriteLine ("Press Enter to quit...");
		Console.ReadLine ();

		// Remove the sample listener when done
		controller.RemoveListener (listener);
		controller.Dispose ();
	}
}
