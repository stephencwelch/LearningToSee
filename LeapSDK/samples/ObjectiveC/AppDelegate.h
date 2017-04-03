/******************************************************************************\
* Copyright (C) 2012-2013 Leap Motion, Inc. All rights reserved.               *
* Leap Motion proprietary and confidential. Not for distribution.              *
* Use subject to the terms of the Leap Motion SDK Agreement available at       *
* https://developer.leapmotion.com/sdk_agreement, or another agreement         *
* between Leap Motion and you, your company or other organization.             *
\******************************************************************************/

#import <Cocoa/Cocoa.h>

@class Sample;

@interface AppDelegate : NSObject <NSApplicationDelegate>

@property (nonatomic, strong, readwrite)IBOutlet NSWindow *window;
@property (nonatomic, strong, readwrite)Sample *sample;

@end
