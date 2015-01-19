//
//  ScnVc.h
//  model3d
//
//  Created by Andreas Hauenstein on 2015-01-12.
//  Copyright (c) 2015 AHN. All rights reserved.
//

#import <Cocoa/Cocoa.h>

@interface ScnVc : NSViewController <NSStreamDelegate>

// Buttons to rotate by 45 degrees around one axis
- (IBAction)btn10045:(id)sender; // x
- (IBAction)btn01045:(id)sender; // y
- (IBAction)btn00145:(id)sender; // z

// Button rotating to one particular orientation
- (IBAction)btnto10045:(id)sender;

- (void) stopListening;

@end
