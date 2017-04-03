/******************************************************************************\
* Copyright (C) 2012-2013 Leap Motion, Inc. All rights reserved.               *
* Leap Motion proprietary and confidential. Not for distribution.              *
* Use subject to the terms of the Leap Motion SDK Agreement available at       *
* https://developer.leapmotion.com/sdk_agreement, or another agreement         *
* between Leap Motion and you, your company or other organization.             *
\******************************************************************************/

#import "AppDelegate.h"
#import "Sample.h"

@implementation AppDelegate

// Xcode 4.2 warns if we do not explicitly synthesize
@synthesize window = _window;
@synthesize sample = _sample; // must retain for notifications

- (void)applicationDidFinishLaunching:(NSNotification *)aNotification
{
    _sample = [[Sample alloc] init];
    [_sample run];
}

@end
